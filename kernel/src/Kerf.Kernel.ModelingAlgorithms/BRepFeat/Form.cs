// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Form.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Form.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Form.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Features.PerfSelection;
using static Kerf.Engine.Features.StatusError;
using static Kerf.Engine.Features.Local.FeatureOperation;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features;

/// <summary>
/// The root of the form features (prisms, draft prisms, revolved features, pipes): the topological reconstruction of
/// the result, by gluing the feature on the basis shape where it can, by a local Boolean operation otherwise; and the
/// history of the basis shape's faces.
/// </summary>
/// <remarks>
/// <para>Each feature built is a working object (decision 21). <see cref="MakeShape.Shape"/>,
/// <see cref="MakeShape.Modified"/>, <see cref="MakeShape.Generated"/>,
/// <see cref="FirstShape"/>, <see cref="LastShape"/>, <see cref="NewEdges"/> and <see cref="TgtEdges"/> publish what
/// they return (frozen); ported code reads the <c>Internal</c> forms. The history answers for the caller's shapes.</para>
/// <para>The basis shape, the sketch face, the from and until shapes are never written: the feature's writes into them
/// (the gluing's, the sweeps', the destructive Booleans') go to its working overlay (decisions 1013, 1052, unit
/// 10H5), and the result holds new TShapes only for the caller's sub-shapes that changed.</para>
/// </remarks>
public abstract class Form : MakeShape
{
    protected bool _fuse;
    protected bool _modify;
    protected Dictionary<Shape, NCollection_List<Shape>> _map = new(ShapeHasher.Instance);
    protected Shape _fShape = new();
    protected Shape _lShape = new();
    protected NCollection_List<Shape> _newEdges = new();
    protected NCollection_List<Shape> _tgtEdges = new();
    protected PerfSelection _perfSelection;
    protected bool _justGluer;
    protected bool _justFeat;
    protected Shape _sbase = new();
    protected Face _skface = new();
    protected Shape _gShape = new();
    protected Shape _sFrom = new();
    protected Shape _sUntil = new();
    protected Dictionary<Shape, Shape> _gluedF = new(ShapeHasher.Instance);

    private bool _sbOK;
    private bool _skOK;
    private bool _gsok;
    private bool _sfok;
    private bool _suok;
    private bool _gfok;
    private bool _psok;
    private StatusError _statusError;

    // Kerf: the working overlay the feature writes into the shapes it is given in (decision 1052, unit 10H5)
    private protected WorkingShapes? _work;
    private WorkingShapes? _finishedWork;

    // OCCT.NET: OCCT's IsSame(sh, F) in the history, where sh may stand for the caller's F changed in place: OCCT
    // changes F in place, keeping its identity, so what stands for it counts as F (decision 1052)
    private bool IsSameInHistory(in Shape sh, in Shape f) =>
        sh.IsSame(f) || (_finishedWork is not null && sh.IsSame(_finishedWork.Final(f)));

    /// <summary>Redefines the empty constructor.</summary>
    protected Form()
    {
        _fuse = false;
        _modify = false;
        _perfSelection = PerfSelection.NoSelection;
        _justGluer = false;
        _justFeat = false;
        _sbOK = false;
        _skOK = false;
        _gsok = false;
        _sfok = false;
        _suok = false;
        _gfok = false;
        _psok = false;
        _statusError = StatusError.NotInitialized;
    }

    /// <summary>Initializes the topological construction if the basis shape is present.</summary>
    public void BasisShapeValid() => _sbOK = true;

    /// <summary>Initializes the topological construction if the generated shape is present.</summary>
    public void GeneratedShapeValid() => _gsok = true;

    /// <summary>Initializes the topological construction if the shape is present from the specified integer on.</summary>
    public void ShapeFromValid() => _sfok = true;

    /// <summary>Initializes the topological construction if the shape is present until the specified integer.</summary>
    public void ShapeUntilValid() => _suok = true;

    /// <summary>Initializes the topological construction if the glued face is present.</summary>
    public void GluedFacesValid() => _gfok = true;

    /// <summary>
    /// Initializes the topological construction if the sketch face is present. If the sketch face is inside the basis
    /// shape, local operations such as glueing can be performed.
    /// </summary>
    public void SketchFaceValid() => _skOK = true;

    /// <summary>Initializes the topological construction if the selected face is present.</summary>
    public void PerfSelectionValid() => _psok = true;

    /// <summary>The curves the feature is intersected along with the from and until shapes.</summary>
    public abstract void Curves(NCollection_Sequence<Curve3D> s);

    /// <summary>The curve through the barycentre of the feature, along its direction.</summary>
    public abstract Curve3D? BarycCurve();

    /// <summary>The status of the last construction.</summary>
    public StatusError CurrentStatusError() => _statusError;

    /// <summary>
    /// OCCT.NET: starts the feature's working overlay (decision 1052, unit 10H5). OCCT's feature writes into the shapes
    /// it is given: the Booleans it runs are destructive (they raise tolerances and add pcurves in place) and the sweeps
    /// write into the profile. The feature works on the caller's shapes in a detached working overlay, which each of
    /// its public calls that runs the construction enters (<see cref="EnterWork"/>); the result, the history and the
    /// lists hold new TShapes for the caller's sub-shapes it changed, from the first publication
    /// (<see cref="FinishWorkingShapes"/>).
    /// </summary>
    private protected void StartWorkingShapes()
    {
        _work = new WorkingShapes();
        _finishedWork = null;
    }

    /// <summary>Enters the feature's working overlay (unit 10H5), for a public call that runs the construction.</summary>
    private protected WorkingOverlay.Scope EnterWork()
    {
        _work ??= new WorkingShapes();
        return _work.Enter();
    }

    /// <summary>
    /// OCCT.NET: ends the working overlay (decision 1052, unit 10H5): the result and the lists get new TShapes for the
    /// caller's sub-shapes the feature changed; the history is keyed by the caller's shapes. Once per construction;
    /// called by every accessor of the result and the history.
    /// </summary>
    private protected virtual void FinishWorkingShapes()
    {
        if (_work is null || _work.IsFinished)
        {
            return;
        }
        WorkingShapes work = _work;
        _finishedWork = work;
        var shapes = new List<Shape> { _shape };
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> item in _map)
        {
            shapes.AddRange(item.Value);
        }
        shapes.AddRange(_newEdges);
        shapes.AddRange(_tgtEdges);
        work.Finish(shapes);
        if (!work.IsWritten)
        {
            return;
        }
        _shape = work.Final(_shape);
        foreach (Shape key in _map.Keys.ToList())
        {
            _map.GetValueRef(key) = work.Final(_map.GetValue(key));
        }
        _newEdges = work.Final(_newEdges);
        _tgtEdges = work.Final(_tgtEdges);
    }

    /// <summary>The result, unfrozen, on the caller's shapes where they are unchanged (OCCT's virtual).</summary>
    internal override Shape InternalShape()
    {
        base.InternalShape();
        FinishWorkingShapes();
        return _shape;
    }

    /// <summary>General perform method: the topological reconstruction of the result.</summary>
    protected void GlobalPerform()
    {
        if (!_sbOK || !_gsok || !_sfok || !_suok || !_gfok || !_skOK || !_psok)
        {
            _statusError = StatusError.NotInitialized;
            NotDone();
            return;
        }

        //--- Initialisation
        Explorer exp = new(), exp2 = new();
        int theOpe = 2;

        if (_justFeat && !_fuse)
        {
            _statusError = StatusError.InvOption;
            NotDone();
            return;
        }
        else if (_justFeat)
        {
            theOpe = 2;
        }
        else if (_gluedF.Count != 0)
        {
            theOpe = 1;
        }
        bool changeOpe = false;

        bool fromInShape = false;
        bool untilInShape = false;

        if (!_sFrom.IsNull)
        {
            fromInShape = true;
            for (exp2.Init(_sFrom, ShapeType.Face); exp2.More(); exp2.Next())
            {
                Shape ffrom = exp2.Current();
                for (exp.Init(_sbase, ShapeType.Face); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(ffrom))
                    {
                        break;
                    }
                }
                if (!exp.More())
                {
                    fromInShape = false;
                    break;
                }
            }
        }

        if (!_sUntil.IsNull)
        {
            untilInShape = true;
            for (exp2.Init(_sUntil, ShapeType.Face); exp2.More(); exp2.Next())
            {
                Shape funtil = exp2.Current();
                for (exp.Init(_sbase, ShapeType.Face); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(funtil))
                    {
                        break;
                    }
                }
                if (!exp.More())
                {
                    untilInShape = false;
                    break;
                }
            }
        }

        NCollection_List<Shape>.Iterator it = new();
        int sens = 0;

        NCollection_Sequence<Curve3D> scur = new();
        Curves(scur);

        double mf, Mf, mu, Mu;

        Orientation orifuntil = Internal;
        Orientation oriffrom = Internal;
        Face fFrom = new(), fUntil = new();

        CSIntersector asi1 = new();
        CSIntersector asi2 = new();

        //--- 1) by intersection

        // Intersection Tool Shape From
        if (!_sFrom.IsNull)
        {
            asi1.Init(_sFrom);
            asi1.Perform(scur!);
        }

        // Intersection Tool Shape Until
        if (!_sUntil.IsNull)
        {
            asi2.Init(_sUntil);
            asi2.Perform(scur!);
        }

        {
            //  Find sens, FFrom, FUntil
            for (int jj = 1; jj <= scur.Length(); jj++)
            {
                if (asi1.IsDone && asi2.IsDone)
                {
                    if (asi1.NbPoints(jj) <= 0)
                    {
                        continue;
                    }
                    mf = asi1.Point(jj, 1).Parameter();
                    Mf = asi1.Point(jj, asi1.NbPoints(jj)).Parameter();
                    if (asi2.NbPoints(jj) <= 0)
                    {
                        continue;
                    }
                    mu = asi2.Point(jj, 1).Parameter();
                    Mu = asi2.Point(jj, asi2.NbPoints(jj)).Parameter();
                    if (!scur[jj].IsPeriodic)
                    {
                        int ku, kf;
                        if (mu <= Mf && mf <= Mu)
                        { // overlapping intervals
                            sens = 1;
                            kf = 1;
                            ku = asi2.NbPoints(jj);
                        }
                        else if (mu > Mf)
                        {
                            if (sens == -1)
                            {
                                _statusError = StatusError.IntervalOverlap;
                                NotDone();
                                return;
                            }
                            sens = 1;
                            kf = 1;
                            ku = asi2.NbPoints(jj);
                        }
                        else
                        {
                            if (sens == 1)
                            {
                                _statusError = StatusError.IntervalOverlap;
                                NotDone();
                                return;
                            }
                            sens = -1;
                            kf = asi1.NbPoints(jj);
                            ku = 1;
                        }
                        if (oriffrom == Internal)
                        {
                            Orientation oript = asi1.Point(jj, kf).Orientation();
                            if (oript == Forward || oript == Reversed)
                            {
                                if (sens == -1)
                                {
                                    oript = global::Kerf.Engine.Topology.TopAbs.Reverse(oript);
                                }
                                oriffrom = global::Kerf.Engine.Topology.TopAbs.Reverse(oript);
                                fFrom = asi1.Point(jj, kf).Face();
                            }
                        }
                        if (orifuntil == Internal)
                        {
                            Orientation oript = asi2.Point(jj, ku).Orientation();
                            if (oript == Forward || oript == Reversed)
                            {
                                if (sens == -1)
                                {
                                    oript = global::Kerf.Engine.Topology.TopAbs.Reverse(oript);
                                }
                                orifuntil = oript;
                                fUntil = asi2.Point(jj, ku).Face();
                            }
                        }
                    }
                }
                else if (asi2.IsDone)
                {
                    if (asi2.NbPoints(jj) <= 0)
                    {
                        continue;
                    }

                    // for base case prism on mySUntil -> ambivalent direction
                    //      ->  preferable direction = 1
                    if (sens != 1)
                    {
                        if (asi2.Point(jj, 1).Parameter() * asi2.Point(jj, asi2.NbPoints(jj)).Parameter() <= 0)
                        {
                            sens = 1;
                        }
                        else if (asi2.Point(jj, 1).Parameter() < 0.0)
                        {
                            sens = -1;
                        }
                        else
                        {
                            sens = 1;
                        }
                    }

                    int ku;
                    if (sens == -1)
                    {
                        ku = 1;
                    }
                    else
                    {
                        ku = asi2.NbPoints(jj);
                    }
                    if (orifuntil == Internal && sens != 0)
                    {
                        Orientation oript = asi2.Point(jj, ku).Orientation();
                        if (oript == Forward || oript == Reversed)
                        {
                            if (sens == -1)
                            {
                                oript = global::Kerf.Engine.Topology.TopAbs.Reverse(oript);
                            }
                            orifuntil = oript;
                            fUntil = asi2.Point(jj, ku).Face();
                        }
                    }
                }
                else
                {
                    sens = 1;
                    break;
                }
            }
        }

        LocalGluer theGlue = new();

        //--- case of gluing

        if (theOpe == 1)
        {
            bool collage = true;
            // cut by FFrom && FUntil
            Compound comp = new();
            BrepBuilder b = new();
            b.MakeCompound(ref comp);
            if (!_sFrom.IsNull)
            {
                Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sFrom, fFrom, oriffrom);
                if (!s.IsNull)
                {
                    b.Add(comp, s);
                }
            }
            if (!_sUntil.IsNull)
            {
                Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, orifuntil);
                if (!s.IsNull)
                {
                    b.Add(comp, s);
                }
            }

            FindEdges theFE = new();
            Dictionary<Shape, NCollection_List<Shape>> locmap = new(ShapeHasher.Instance);
            Explorer expp = new(comp, ShapeType.Solid);
            if (expp.More() && !comp.IsNull && !_gShape.IsNull)
            {
                Cut trP = new(_gShape, comp);
                exp.Init(trP.InternalShape(), ShapeType.Solid);
                if (exp.Current().IsNull)
                {
                    theOpe = 2;
                    changeOpe = true;
                    collage = false;
                }
                else
                { // else X0
                    // Only solids are preserved
                    Compound theGShape = new();
                    b.MakeCompound(ref theGShape);
                    for (; exp.More(); exp.Next())
                    {
                        b.Add(theGShape, exp.Current());
                    }
                    if (!global::Kerf.Engine.Booleans.Algo.BRepAlgo.IsValid(theGShape))
                    {
                        theOpe = 2;
                        changeOpe = true;
                        collage = false;
                    }
                    else
                    { // else X1
                        if (!_sFrom.IsNull)
                        {
                            Explorer ex = new();
                            ex.Init(_sFrom, ShapeType.Face);
                            for (; ex.More(); ex.Next())
                            {
                                Face fac = ShapeCasts.Face(ex.Current());
                                if (!fromInShape)
                                {
                                    NCollection_List<Shape> thelist = new();
                                    _map[fac] = thelist;
                                }
                                else
                                {
                                    NCollection_List<Shape> thelist1 = new();
                                    locmap[fac] = thelist1;
                                }
                                if (trP.IsDeleted(fac))
                                {
                                }
                                else if (!fromInShape)
                                {
                                    _map.GetValueRef(fac) = new NCollection_List<Shape>(trP.InternalModified(fac));
                                    if (_map.GetValue(fac).IsEmpty())
                                    {
                                        _map.GetValue(fac).Append(fac);
                                    }
                                }
                                else
                                {
                                    locmap.GetValueRef(fac) = new NCollection_List<Shape>(trP.InternalModified(fac));
                                    if (locmap.GetValue(fac).IsEmpty())
                                    {
                                        locmap.GetValue(fac).Append(fac);
                                    }
                                }
                            }
                        } // if(!mySFrom.IsNull())
                        //
                        if (!_sUntil.IsNull)
                        {
                            Explorer ex = new();
                            ex.Init(_sUntil, ShapeType.Face);
                            for (; ex.More(); ex.Next())
                            {
                                Face fac = ShapeCasts.Face(ex.Current());
                                if (!untilInShape)
                                {
                                    NCollection_List<Shape> thelist2 = new();
                                    _map[fac] = thelist2;
                                }
                                else
                                {
                                    NCollection_List<Shape> thelist3 = new();
                                    locmap[fac] = thelist3;
                                }
                                if (trP.IsDeleted(fac))
                                {
                                }
                                else if (!untilInShape)
                                {
                                    _map.GetValueRef(fac) = new NCollection_List<Shape>(trP.InternalModified(fac));
                                    if (_map.GetValue(fac).IsEmpty())
                                    {
                                        _map.GetValue(fac).Append(fac);
                                    }
                                }
                                else
                                {
                                    locmap.GetValueRef(fac) = new NCollection_List<Shape>(trP.InternalModified(fac));
                                    if (locmap.GetValue(fac).IsEmpty())
                                    {
                                        locmap.GetValue(fac).Append(fac);
                                    }
                                }
                            }
                        } // if(!mySUntil.IsNull())
                        //
                        UpdateDescendants(trP, theGShape, true); // skip faces

                        theGlue.Init(_sbase, theGShape);
                        foreach (KeyValuePair<Shape, Shape> itm in _gluedF)
                        {
                            Face gl = ShapeCasts.Face(itm.Key);
                            NCollection_List<Shape> ldsc = new();
                            if (trP.IsDeleted(gl))
                            {
                            }
                            else
                            {
                                ldsc = new NCollection_List<Shape>(trP.InternalModified(gl));
                                if (ldsc.IsEmpty())
                                {
                                    ldsc.Append(gl);
                                }
                            }
                            Face glface = ShapeCasts.Face(itm.Value);
                            for (it.Initialize(ldsc); it.More(); it.Next())
                            {
                                Face fac = ShapeCasts.Face(it.Value());
                                collage = global::Kerf.Engine.Features.BRepFeat.IsInside(fac, glface);
                                if (!collage)
                                {
                                    theOpe = 2;
                                    changeOpe = true;
                                    break;
                                }
                                else
                                {
                                    theGlue.Bind(fac, glface);
                                    theFE.Set(fac, glface);
                                    for (theFE.InitIterator(); theFE.More(); theFE.Next())
                                    {
                                        theGlue.Bind(theFE.EdgeFrom(), theFE.EdgeTo());
                                    }
                                }
                            }
                        }
                    } // else X1
                } // else X0
            } // if(expp.More() && !Comp.IsNull() && !myGShape.IsNull())
            else
            {
                theGlue.Init(_sbase, _gShape);
                foreach (KeyValuePair<Shape, Shape> itm in _gluedF)
                {
                    Face glface = ShapeCasts.Face(itm.Key);
                    Face fac = ShapeCasts.Face(_gluedF.GetValue(glface));
                    for (exp.Init(_gShape, ShapeType.Face); exp.More(); exp.Next())
                    {
                        if (exp.Current().IsSame(glface))
                        {
                            break;
                        }
                    }
                    if (exp.More())
                    {
                        collage = global::Kerf.Engine.Features.BRepFeat.IsInside(glface, fac);
                        if (!collage)
                        {
                            theOpe = 2;
                            changeOpe = true;
                            break;
                        }
                        else
                        {
                            theGlue.Bind(glface, fac);
                            theFE.Set(glface, fac);
                            for (theFE.InitIterator(); theFE.More(); theFE.Next())
                            {
                                theGlue.Bind(theFE.EdgeFrom(), theFE.EdgeTo());
                            }
                        }
                    }
                }
            }

            // Add gluing on start and end face if necessary !!!
            if (fromInShape && collage)
            {
                Explorer ex = new(_sFrom, ShapeType.Face);
                for (; ex.More(); ex.Next())
                {
                    Face fac2 = ShapeCasts.Face(ex.Current());
                    for (it.Initialize(locmap.GetValue(fac2)); it.More(); it.Next())
                    {
                        Face fac1 = ShapeCasts.Face(it.Value());
                        theFE.Set(fac1, fac2);
                        theGlue.Bind(fac1, fac2);
                        for (theFE.InitIterator(); theFE.More(); theFE.Next())
                        {
                            theGlue.Bind(theFE.EdgeFrom(), theFE.EdgeTo());
                        }
                    }
                }
            }

            if (untilInShape && collage)
            {
                Explorer ex = new(_sUntil, ShapeType.Face);
                for (; ex.More(); ex.Next())
                {
                    Face fac2 = ShapeCasts.Face(ex.Current());
                    for (it.Initialize(locmap.GetValue(fac2)); it.More(); it.Next())
                    {
                        Face fac1 = ShapeCasts.Face(it.Value());
                        theGlue.Bind(fac1, fac2);
                        theFE.Set(fac1, fac2);
                        for (theFE.InitIterator(); theFE.More(); theFE.Next())
                        {
                            theGlue.Bind(theFE.EdgeFrom(), theFE.EdgeTo());
                        }
                    }
                }
            }

            FeatureOperation ope = theGlue.OpeType();
            if (ope == FeatureOperation.Invalid || (_fuse && ope != FeatureOperation.Fuse) || (!_fuse && ope != FeatureOperation.Cut) || (!collage))
            {
                theOpe = 2;
                changeOpe = true;
            }
        }

        //--- if the gluing is always applicable

        if (theOpe == 1)
        {
            theGlue.Perform();
            if (theGlue.IsDone)
            {
                Shape shshs = theGlue.InternalResultingShape();
                if (global::Kerf.Engine.Booleans.Algo.BRepAlgo.IsValid(shshs))
                {
                    UpdateDescendants(theGlue);
                    _newEdges = new NCollection_List<Shape>(theGlue.InternalEdges());
                    _tgtEdges = new NCollection_List<Shape>(theGlue.InternalTgtEdges());
                    Done();
                    _shape = theGlue.InternalResultingShape();
                }
                else
                {
                    theOpe = 2;
                    changeOpe = true;
                }
            }
            else
            {
                theOpe = 2;
                changeOpe = true;
            }
        }

        //--- case without gluing + Tool with proper dimensions

        if (theOpe == 2 && changeOpe && _justGluer)
        {
            _justGluer = false;
            theOpe = 0;
        }

        //--- case without gluing

        if (theOpe == 2)
        {
            Shape theGShape = _gShape;

            Compound comp = new();
            BrepBuilder b = new();
            b.MakeCompound(ref comp);
            if (!_sFrom.IsNull || !_sUntil.IsNull)
            {
                if (!_sFrom.IsNull && !fromInShape)
                {
                    Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sFrom, fFrom, oriffrom);
                    if (!s.IsNull)
                    {
                        b.Add(comp, s);
                    }
                }
                if (!_sUntil.IsNull && !untilInShape)
                {
                    if (!_sFrom.IsNull)
                    {
                        if (!_sFrom.IsSame(_sUntil))
                        {
                            Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, orifuntil);
                            if (!s.IsNull)
                            {
                                b.Add(comp, s);
                            }
                        }
                    }
                    else
                    {
                        Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, orifuntil);
                        if (!s.IsNull)
                        {
                            b.Add(comp, s);
                        }
                    }
                }
            }

            // update type of selection
            if (_perfSelection == PerfSelection.SelectionU && !untilInShape)
            {
                _perfSelection = PerfSelection.NoSelection;
            }
            else if (_perfSelection == PerfSelection.SelectionFU && !fromInShape && !untilInShape)
            {
                _perfSelection = PerfSelection.NoSelection;
            }
            else if (_perfSelection == PerfSelection.SelectionShU && !untilInShape)
            {
                _perfSelection = PerfSelection.NoSelection;
            }

            Explorer expp = new(comp, ShapeType.Solid);
            if (expp.More() && !comp.IsNull && !_gShape.IsNull)
            {
                Cut trP = new(_gShape, comp);
                // the result is necessarily a compound.
                exp.Init(trP.InternalShape(), ShapeType.Solid);
                if (!exp.More())
                {
                    _statusError = StatusError.EmptyCutResult;
                    NotDone();
                    return;
                }
                // Only solids are preserved
                Compound gShape = new();
                b.MakeCompound(ref gShape);
                for (; exp.More(); exp.Next())
                {
                    b.Add(gShape, exp.Current());
                }
                theGShape = gShape;
                if (!global::Kerf.Engine.Booleans.Algo.BRepAlgo.IsValid(theGShape))
                {
                    _statusError = StatusError.InvShape;
                    NotDone();
                    return;
                }
                if (!_sFrom.IsNull)
                {
                    if (!fromInShape)
                    {
                        Explorer ex = new(_sFrom, ShapeType.Face);
                        for (; ex.More(); ex.Next())
                        {
                            Face fac = ShapeCasts.Face(ex.Current());
                            NCollection_List<Shape> thelist4 = new();
                            _map[fac] = thelist4;
                            if (trP.IsDeleted(fac))
                            {
                            }
                            else
                            {
                                _map.GetValueRef(fac) = new NCollection_List<Shape>(trP.InternalModified(fac));
                                if (_map.GetValue(fac).IsEmpty())
                                {
                                    _map.GetValue(fac).Append(fac);
                                }
                            }
                        }
                    }
                }
                if (!_sUntil.IsNull)
                {
                    if (!untilInShape)
                    {
                        Explorer ex = new(_sUntil, ShapeType.Face);
                        for (; ex.More(); ex.Next())
                        {
                            Face fac = ShapeCasts.Face(ex.Current());
                            NCollection_List<Shape> thelist5 = new();
                            _map[fac] = thelist5;
                            if (trP.IsDeleted(fac))
                            {
                            }
                            else
                            {
                                _map.GetValueRef(fac) = new NCollection_List<Shape>(trP.InternalModified(fac));
                                if (_map.Count == 0) // OCCT: tests the map, not the face's list
                                {
                                    _map.GetValue(fac).Append(fac);
                                }
                            }
                        }
                    }
                }
                UpdateDescendants(trP, theGShape, true);
            } // if(expp.More() && !Comp.IsNull() && !myGShape.IsNull())  {
            //

            //--- generation of "just feature" for assembly = Parts of tool
            bool bFlag = _perfSelection != PerfSelection.NoSelection;
            FeatureBuilder theBuilder = new();
            theBuilder.Init(_sbase, theGShape);
            theBuilder.SetOperation(_fuse ? 1 : 0, bFlag);
            theBuilder.Perform();
            //
            NCollection_List<Shape> lshape = new();
            theBuilder.PartsOfTool(lshape);
            //
            double pbmin = RealLast(), pbmax = RealFirst();
            double prmin = RealLast() - 2 * global::Kerf.Engine.Foundation.Precision.Confusion();
            double prmax = RealFirst() + 2 * global::Kerf.Engine.Foundation.Precision.Confusion();
            bool flag1 = false;
            Curve3D? c = null;

            //--- Selection of pieces of tool to be preserved
            if (!lshape.IsEmpty() && _perfSelection != PerfSelection.NoSelection)
            {
                //      Find ParametricMinMax depending on the constraints of Shape From and Until
                //   -> prmin, prmax, pbmin and pbmax
                c = BarycCurve();
                if (c is null)
                {
                    _statusError = StatusError.EmptyBaryCurve;
                    NotDone();
                    return;
                }

                if (_perfSelection == PerfSelection.SelectionSh)
                {
                    global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(_sbase, c, out prmin, out prmax, out pbmin, out pbmax, out flag1);
                }
                else if (_perfSelection == PerfSelection.SelectionFU)
                {
                    global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(_sFrom, c, out double prmin1, out double prmax1, out double prbmin1, out double prbmax1, out flag1);
                    global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(_sUntil, c, out double prmin2, out double prmax2, out double prbmin2, out double prbmax2, out flag1);

                    // case of revolutions
                    if (c.IsPeriodic)
                    {
                        double period = c.Period;
                        prmax = prmax2;
                        if (flag1)
                        {
                            prmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(prmin1, prmax - period, prmax);
                        }
                        else
                        {
                            prmin = Min(prmin1, prmin2);
                        }
                        pbmax = prbmax2;
                        pbmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(prbmin1, pbmax - period, pbmax);
                    }
                    else
                    {
                        prmin = Min(prmin1, prmin2);
                        prmax = Max(prmax1, prmax2);
                        pbmin = Min(prbmin1, prbmin2);
                        pbmax = Max(prbmax1, prbmax2);
                    }
                }
                else if (_perfSelection == PerfSelection.SelectionShU)
                {
                    if (!_justFeat && sens == 0)
                    {
                        sens = 1;
                    }
                    if (sens == 0)
                    {
                        _statusError = StatusError.IncDirection;
                        NotDone();
                        return;
                    }

                    global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(_sUntil, c, out double prmin1, out double prmax1, out double prbmin1, out double prbmax1, out flag1);

                    global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(_sbase, c, out double prmin2, out double prmax2, out double prbmin2, out double prbmax2, out flag1);
                    if (sens == 1)
                    {
                        prmin = prmin2;
                        prmax = prmax1;
                        pbmin = prbmin2;
                        pbmax = prbmax1;
                    }
                    else if (sens == -1)
                    {
                        prmin = prmin1;
                        prmax = prmax2;
                        pbmin = prbmin1;
                        pbmax = prbmax2;
                    }
                }
                else if (_perfSelection == PerfSelection.SelectionU)
                {
                    if (sens == 0)
                    {
                        _statusError = StatusError.IncDirection;
                        NotDone();
                        return;
                    }

                    // Find parts of the tool containing descendants of Shape Until
                    global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(_sUntil, c, out double prmin1, out double prmax1, out double prbmin1, out double prbmax1, out flag1);
                    if (sens == 1)
                    {
                        prmin = RealFirst();
                        prmax = prmax1;
                        pbmin = RealFirst();
                        pbmax = prbmax1;
                    }
                    else if (sens == -1)
                    {
                        prmin = prmin1;
                        prmax = RealLast();
                        pbmin = prbmin1;
                        pbmax = RealLast();
                    }
                }

                // Finer choice of ParametricMinMax in case when the tool
                // intersects Shapes From and Until
                //       case of several intersections (keep PartsOfTool according to the selection)
                //       position of the face of intersection in PartsOfTool (before or after)
                double delta = global::Kerf.Engine.Foundation.Precision.Confusion();

                if (_perfSelection != PerfSelection.NoSelection)
                {
                    // modif of the test for cts21181 : (prbmax2 and prnmin2) -> (prbmin1 and prbmax1)
                    // correction take into account flag2 for pro15323 and flag3 for pro16060
                    if (!_sUntil.IsNull)
                    {
                        HashSet<Shape> mapFuntil = new(ShapeHasher.Instance);
                        Descendants(_sUntil, theBuilder, mapFuntil);
                        if (mapFuntil.Count != 0)
                        {
                            for (it.Initialize(lshape); it.More(); it.Next())
                            {
                                Explorer expf = new();
                                for (expf.Init(it.Value(), ShapeType.Face); expf.More(); expf.Next())
                                {
                                    if (mapFuntil.Contains(expf.Current()))
                                    {
                                        global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(expf.Current(), c, out double prmin1, out double prmax1, out double prbmin1, out double prbmax1, out bool flag3);
                                        global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(it.Value(), c, out double prmin2, out double prmax2, out double prbmin2, out double prbmax2, out bool flag2);
                                        if (sens == 1)
                                        {
                                            bool testOK = !flag2;
                                            if (flag2)
                                            {
                                                testOK = !flag1;
                                                if (flag1 && prmax2 > prmin + delta)
                                                {
                                                    testOK = !flag3;
                                                    if (flag3 && prmax1 == prmax2)
                                                    {
                                                        testOK = true;
                                                    }
                                                }
                                            }
                                            if (prbmin1 < pbmax && testOK)
                                            {
                                                if (flag2)
                                                {
                                                    flag1 = flag2;
                                                    prmax = prmax2;
                                                }
                                                pbmax = prbmin1;
                                            }
                                        }
                                        else if (sens == -1)
                                        {
                                            bool testOK = !flag2;
                                            if (flag2)
                                            {
                                                testOK = !flag1;
                                                if (flag1 && prmin2 < prmax - delta)
                                                {
                                                    testOK = !flag3;
                                                    if (flag3 && prmin1 == prmin2)
                                                    {
                                                        testOK = true;
                                                    }
                                                }
                                            }
                                            if (prbmax1 > pbmin && testOK)
                                            {
                                                if (flag2)
                                                {
                                                    flag1 = flag2;
                                                    prmin = prmin2;
                                                }
                                                pbmin = prbmax1;
                                            }
                                        }
                                        break;
                                    }
                                }
                            }
                            it.Initialize(lshape);
                        }
                    }
                    if (!_sFrom.IsNull)
                    {
                        HashSet<Shape> mapFfrom = new(ShapeHasher.Instance);
                        Descendants(_sFrom, theBuilder, mapFfrom);
                        if (mapFfrom.Count != 0)
                        {
                            for (it.Initialize(lshape); it.More(); it.Next())
                            {
                                Explorer expf = new();
                                for (expf.Init(it.Value(), ShapeType.Face); expf.More(); expf.Next())
                                {
                                    if (mapFfrom.Contains(expf.Current()))
                                    {
                                        global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(expf.Current(), c, out double prmin1, out double prmax1, out double prbmin1, out double prbmax1, out bool flag3);
                                        global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(it.Value(), c, out double prmin2, out double prmax2, out double prbmin2, out double prbmax2, out bool flag2);
                                        if (sens == 1)
                                        {
                                            bool testOK = !flag2;
                                            if (flag2)
                                            {
                                                testOK = !flag1;
                                                if (flag1 && prmin2 < prmax - delta)
                                                {
                                                    testOK = !flag3;
                                                    if (flag3 && prmin1 == prmin2)
                                                    {
                                                        testOK = true;
                                                    }
                                                }
                                            }
                                            if (prbmax1 > pbmin && testOK)
                                            {
                                                if (flag2)
                                                {
                                                    flag1 = flag2;
                                                    prmin = prmin2;
                                                }
                                                pbmin = prbmax1;
                                            }
                                        }
                                        else if (sens == -1)
                                        {
                                            bool testOK = !flag2;
                                            if (flag2)
                                            {
                                                testOK = !flag1;
                                                if (flag1 && prmax2 > prmin + delta)
                                                {
                                                    testOK = !flag3;
                                                    if (flag3 && prmax1 == prmax2)
                                                    {
                                                        testOK = true;
                                                    }
                                                }
                                            }
                                            if (prbmin1 < pbmax && testOK)
                                            {
                                                if (flag2)
                                                {
                                                    flag1 = flag2;
                                                    prmax = prmax2;
                                                }
                                                pbmax = prbmin1;
                                            }
                                        }
                                        break;
                                    }
                                }
                            }
                            it.Initialize(lshape);
                        }
                    }
                }

                // Parse PartsOfTool to preserve or not depending on ParametricMinMax
                if (!_justFeat)
                {
                    bool keepParts = false;
                    double prmin1, prmax1, prbmin1, prbmax1;
                    double min, max, pmin, pmax;
                    bool flag2;
                    for (it.Initialize(lshape); it.More(); it.Next())
                    {
                        if (c.IsPeriodic)
                        {
                            double period = c.Period;
                            global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(it.Value(), c, out double pr, out prmax1, out double prb, out prbmax1, out flag2, true);
                            if (flag2)
                            {
                                prmin1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(pr, prmax1 - period, prmax1);
                            }
                            else
                            {
                                prmin1 = pr;
                            }
                            prbmin1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(prb, prbmax1 - period, prbmax1);
                        }
                        else
                        {
                            global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(it.Value(), c, out prmin1, out prmax1, out prbmin1, out prbmax1, out flag2);
                        }
                        if (!flag2 || !flag1)
                        {
                            pmin = pbmin;
                            pmax = pbmax;
                            min = prbmin1;
                            max = prbmax1;
                        }
                        else
                        {
                            pmin = prmin;
                            pmax = prmax;
                            min = prmin1;
                            max = prmax1;
                        }
                        if ((min <= pmax - delta) && (max >= pmin + delta))
                        {
                            keepParts = true;
                            Shape s = it.Value();
                            theBuilder.KeepPart(s);
                        }
                    }

                    // Case when no part of the tool is preserved
                    if (!keepParts)
                    {
                        _statusError = StatusError.NoParts;
                        NotDone();
                        return;
                    }
                }
                else
                {
                    // case JustFeature -> all PartsOfTool are preserved
                    double prmin1, prmax1, prbmin1, prbmax1;
                    double min, max, pmin, pmax;
                    bool flag2;
                    Compound compo = new();
                    b.MakeCompound(ref compo);
                    for (it.Initialize(lshape); it.More(); it.Next())
                    {
                        global::Kerf.Engine.Features.BRepFeat.ParametricMinMax(it.Value(), c, out prmin1, out prmax1, out prbmin1, out prbmax1, out flag2);
                        if (!flag2 || !flag1)
                        {
                            pmin = pbmin;
                            pmax = pbmax;
                            min = prbmin1;
                            max = prbmax1;
                        }
                        else
                        {
                            pmin = prmin;
                            pmax = prmax;
                            min = prmin1;
                            max = prmax1;
                        }
                        if ((min < pmax - delta) && (max > pmin + delta))
                        {
                            if (!it.Value().IsNull)
                            {
                                b.Add(compo, it.Value());
                            }
                        }
                    }
                    _shape = compo;
                }
            }

            //--- Generation of result myShape

            if (!_justFeat)
            {
                // removal of edges of section that have no common vertices
                // with PartsOfTool preserved
                if (bFlag)
                {
                    theBuilder.PerformResult();
                    _shape = theBuilder.InternalShape();
                }
                else
                {
                    _shape = theBuilder.InternalShape();
                }
                Done();
            }
            else
            {
                // all is already done
                Done();
            }
        }

        _statusError = StatusError.OK;
    }

    /// <summary>True if the face <paramref name="f"/> of the basis shape has no descendant in the result.</summary>
    public override bool IsDeleted(in Shape f)
    {
        FinishWorkingShapes();
        if (_map.ContainsKey(f))
        {
            return _map.GetValue(f).IsEmpty();
        }
        return false;
    }

    /// <summary>The faces generated from the face <paramref name="f"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        FinishWorkingShapes();
        _generated.Clear();
        if (!IsDone)
        {
            return _generated;
        }

        if (_sbase.IsEqual(f))
        {
            _generated.Append(_shape);
            return _generated;
        }

        if (_map.ContainsKey(f))
        {
            NCollection_List<Shape>.Iterator ite = new(_map.GetValue(f));
            for (; ite.More(); ite.Next())
            {
                Shape sh = ite.Value();
                if (!IsSameInHistory(sh, f) && sh.Type == f.Type)
                {
                    _generated.Append(sh);
                }
            }
        }
        return _generated; // empty list
    }

    /// <summary>The faces created from the shape <paramref name="s"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        FinishWorkingShapes();
        _generated.Clear();
        if (!IsDone)
        {
            return _generated;
        }
        if (_map.ContainsKey(s) && s.Type != ShapeType.Face)
        { // check if filter on face or not
            NCollection_List<Shape>.Iterator ite = new(_map.GetValue(s));
            for (; ite.More(); ite.Next())
            {
                Shape sh = ite.Value();
                if (!IsSameInHistory(sh, s))
                {
                    _generated.Append(sh);
                }
            }
            return _generated;
        }
        return _generated;
    }

    // The history's lists hold sub-shapes of the result: publishing them publishes the result (decision 414).
    private protected override void BeforePublishingLists()
    {
        FinishWorkingShapes();
        _shape.TShape()?.FreezeGraph();
    }

    protected void UpdateDescendants(LocalGluer g)
    {
        NCollection_List<Shape>.Iterator it = new(), it2 = new();

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in _map)
        {
            Shape orig = itdm.Key;
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 1054)
            IndexedSet<Shape> newdsc = new(ShapeHasher.Instance);
            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                Face fdsc = ShapeCasts.Face(it.Value());
                for (it2.Initialize(g.InternalDescendantFaces(fdsc)); it2.More(); it2.Next())
                {
                    newdsc.Add(it2.Value());
                }
            }
            _map.GetValue(orig).Clear();
            foreach (Shape itm in newdsc)
            {
                _map.GetValue(orig).Append(itm);
            }
        }
    }

    /// <summary>The shapes created at the bottom of the created form; it may be empty (frozen).</summary>
    public ISchematiqList<Shape> FirstShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalFirstShape());
    }

    /// <summary><see cref="FirstShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalFirstShape()
    {
        FinishWorkingShapes();
        if (!_fShape.IsNull)
        {
            return _map.GetValue(_fShape);
        }
        return _generated; // empty list
    }

    /// <summary>The shapes created at the top of the created form; it may be empty (frozen).</summary>
    public ISchematiqList<Shape> LastShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalLastShape());
    }

    /// <summary><see cref="LastShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalLastShape()
    {
        FinishWorkingShapes();
        if (!_lShape.IsNull)
        {
            return _map.GetValue(_lShape);
        }
        return _generated; // empty list
    }

    /// <summary>
    /// The limiting and glueing edges generated by the feature, which did not exist in the basis shape: the
    /// information needed to add fillets afterwards. It may be empty (frozen).
    /// </summary>
    public ISchematiqList<Shape> NewEdges()
    {
        BeforePublishingLists();
        return PublishedList(InternalNewEdges());
    }

    /// <summary><see cref="NewEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalNewEdges()
    {
        FinishWorkingShapes();
        return _newEdges;
    }

    /// <summary>
    /// The tangent edges among <see cref="NewEdges"/>: no fillet is possible on them. It may be empty (frozen).
    /// </summary>
    public ISchematiqList<Shape> TgtEdges()
    {
        BeforePublishingLists();
        return PublishedList(InternalTgtEdges());
    }

    /// <summary><see cref="TgtEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalTgtEdges()
    {
        FinishWorkingShapes();
        return _tgtEdges;
    }

    /// <summary>Limitation of the from (flag 0) or until (flag 1) shape in the case of infinite faces.</summary>
    protected bool TransformShapeFU(int flag)
    {
        bool trf = false;

        Shape shapefu;
        if (flag == 0)
        {
            shapefu = _sFrom;
        }
        else if (flag == 1)
        {
            shapefu = _sUntil;
        }
        else
        {
            return trf;
        }

        Explorer exp = new(shapefu, ShapeType.Face);
        if (!exp.More())
        { // no faces... It is necessary to return an error
            return trf;
        }

        exp.Next();
        if (!exp.More())
        { // the only face. Is it infinite?
            exp.ReInit();
            Face fac = ShapeCasts.Face(exp.Current());

            Surface s = Tool.Surface(fac)!;
            if (s is TrimmedSurface trimmed)
            {
                s = trimmed.BasisSurface;
            }

            if (s.GetType() == typeof(PlaneSurface) || s.GetType() == typeof(CylindricalSurface)
                || s.GetType() == typeof(ConicalSurface))
            {
                Explorer exp1 = new(fac, ShapeType.Wire);
                if (!exp1.More())
                {
                    trf = true;
                }
                else
                {
                    trf = Tool.NaturalRestriction(fac);
                }
            }
            if (trf)
            {
                global::Kerf.Engine.Features.BRepFeat.FaceUntil(_sbase, ref fac);
            }

            if (flag == 0)
            {
                NCollection_List<Shape> thelist6 = new();
                _map[_sFrom] = thelist6;
                _map.GetValue(_sFrom).Append(fac);
                _sFrom = fac;
            }
            else if (flag == 1)
            {
                NCollection_List<Shape> thelist7 = new();
                _map[_sUntil] = thelist7;
                _map.GetValue(_sUntil).Append(fac);
                _sUntil = fac;
            }
        }
        else
        {
            for (exp.ReInit(); exp.More(); exp.Next())
            {
                Shape fac = exp.Current();
                NCollection_List<Shape> thelist8 = new();
                _map[fac] = thelist8;
                _map.GetValue(fac).Append(fac);
            }
        }
        return trf;
    }

    private static void Descendants(in Shape s, FeatureBuilder fb, HashSet<Shape> mapF)
    {
        mapF.Clear();
        Explorer exp = new();
        for (exp.Init(s, ShapeType.Face); exp.More(); exp.Next())
        {
            Face fdsc = ShapeCasts.Face(exp.Current());
            foreach (Shape aS in fb.Modified(fdsc))
            {
                mapF.Add(aS);
            }
        }
    }

    protected void UpdateDescendants(BooleanOperation aBOP, in Shape s, bool skipFace = false)
    {
        NCollection_List<Shape>.Iterator it = new(), it2 = new();
        Explorer exp = new();

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in _map)
        {
            Shape orig = itdm.Key;
            if (skipFace && orig.Type == ShapeType.Face)
            {
                continue;
            }
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 1054)
            IndexedSet<Shape> newdsc = new(ShapeHasher.Instance);

            if (itdm.Value.IsEmpty())
            {
                _map.GetValue(orig).Append(orig);
            }

            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                Shape sh = it.Value();
                if (sh.Type != ShapeType.Face)
                {
                    continue;
                }
                Face fdsc = ShapeCasts.Face(it.Value());
                for (exp.Init(s, ShapeType.Face); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(fdsc))
                    { // preserved
                        newdsc.Add(fdsc);
                        break;
                    }
                }
                if (!exp.More())
                {
                    NCollection_List<Shape> lm = aBOP.InternalModified(fdsc);
                    it2.Initialize(lm);
                    for (; it2.More(); it2.Next())
                    {
                        Shape aS = it2.Value();
                        newdsc.Add(aS);
                    }
                }
            }
            _map.GetValue(orig).Clear();
            foreach (Shape itm in newdsc)
            {
                // check the appartenance to the shape...
                for (exp.Init(s, ShapeType.Face); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(itm))
                    {
                        _map.GetValue(orig).Append(itm);
                        break;
                    }
                }
            }
        }
    }
}
