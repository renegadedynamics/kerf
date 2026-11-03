// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_Modification.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_Modification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Features.Draft.DraftErrorStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features.Draft;

/// <summary>
/// The draft angle modification of a shape for <see cref="Modifier"/>: faces turned by an angle around their
/// intersection with a neutral plane, with their tangent neighbours, and the new edges and vertices between them.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shape it is given: the new geometry it computes is its own, and
/// <see cref="NewCurve2d"/> writes only the range of the modifier's new edge. OCCT's maps keyed by faces, edges and
/// vertices with TopTools_ShapeMapHasher are keyed by TopoDS_Shape in the port (the same IsSame comparison).
/// </remarks>
public sealed partial class DraftModification : Modification
{
    private readonly IndexedMap<Shape, DraftFaceInfo> _fMap = new(ShapeHasher.Instance);
    private readonly IndexedMap<Shape, DraftEdgeInfo> _eMap = new(ShapeHasher.Instance);
    private readonly IndexedMap<Shape, VertexInfo> _vMap = new(ShapeHasher.Instance);
    private bool _comp;
    private Shape _shape = new();
    private Shape _badShape = new();
    private DraftErrorStatus _errStat;
    private Face _curFace = new();
    private readonly NCollection_List<Shape> _conneF = new();
    private readonly IndexedMap<Shape, NCollection_List<Shape>> _efMap =
        new(ShapeHasher.Instance);

    //=================================================================================================

    /// <summary>The modification of the shape <paramref name="s"/>.</summary>
    public DraftModification(in Shape s)
    {
        _comp = false;
        _shape = s;
        Exploration.MapShapesAndAncestors(_shape, ShapeType.Edge, ShapeType.Face, _efMap);
    }

    //=================================================================================================

    /// <summary>Resets on the same shape.</summary>
    public void Clear()
    {
        _comp = false;
        _fMap.Clear();
        _eMap.Clear();
        _vMap.Clear();
        _efMap.Clear();
        _badShape = new Shape();
        _errStat = DraftErrorStatus.NoError;
    }

    //=================================================================================================

    /// <summary>Changes the basis shape and resets.</summary>
    public void Init(in Shape s)
    {
        _shape = s;
        Clear();
        Exploration.MapShapesAndAncestors(_shape, ShapeType.Edge, ShapeType.Face, _efMap);
    }

    //=================================================================================================

    /// <summary>
    /// Adds the face <paramref name="f"/> and propagates the draft to its tangent neighbours. Returns false on an
    /// error; <see cref="ProblematicShape"/> is then the bad shape.
    /// </summary>
    /// <exception cref="ConstructionErrorException">An earlier Add failed and the bad face wasn't removed.</exception>
    public bool Add(in Face f, in Direction3D direction, double angle, in Plane3D neutralPlane, bool flag = true)
    {
        if (!_badShape.IsNull)
        {
            throw new ConstructionErrorException();
        }

        if (_comp)
        {
            Clear();
        }
        _curFace = f;
        return InternalAdd(f, direction, angle, neutralPlane, flag);
    }

    //=================================================================================================

    /// <summary>Removes the face <paramref name="f"/> and the faces added with it.</summary>
    /// <exception cref="NoSuchObjectException">The face wasn't added, or the modification was performed.</exception>
    public void Remove(in Face f)
    {
        if (!_fMap.ContainsKey(f) || _comp)
        {
            throw new NoSuchObjectException();
        }

        _conneF.Clear();

        _curFace = _fMap.GetValue(f).RootFace();
        for (int i = 0; i < _fMap.Count; i++)
        {
            Face theF = ShapeCasts.Face(_fMap.KeyAt(i));
            if (_fMap.GetValue(theF).RootFace().IsSame(_curFace))
            {
                _conneF.Append(theF);
                if (theF.IsSame(_badShape))
                {
                    _badShape = new Shape();
                }
            }
        }

        for (NCollection_List<Shape>.Iterator ltod = new(_conneF); ltod.More(); ltod.Next())
        {
            _fMap.Remove(ShapeCasts.Face(ltod.Value()));
        }

        _conneF.Clear();
        for (int i = 0; i < _eMap.Count; i++)
        {
            Edge theE = ShapeCasts.Edge(_eMap.KeyAt(i));
            if (_eMap.GetValue(theE).RootFace().IsSame(_curFace))
            {
                _conneF.Append(theE);
            }
        }
        for (NCollection_List<Shape>.Iterator ltod = new(_conneF); ltod.More(); ltod.Next())
        {
            _eMap.Remove(ShapeCasts.Edge(ltod.Value()));
        }
    }

    //=================================================================================================

    /// <summary>True if <see cref="Perform"/> succeeded.</summary>
    public bool IsDone => _comp && _badShape.IsNull;

    //=================================================================================================

    /// <summary>The error status.</summary>
    public DraftErrorStatus Error() => _errStat;

    //=================================================================================================

    /// <summary>The face, edge or vertex on which an error occurred (null if none).</summary>
    public Shape ProblematicShape() => _badShape;

    //=================================================================================================

    /// <summary>The faces added together with the face <paramref name="f"/>.</summary>
    /// <exception cref="NoSuchObjectException">The face wasn't added.</exception>
    /// <exception cref="NotDoneException">The modification isn't done.</exception>
    public NCollection_List<Shape> ConnectedFaces(in Face f)
    {
        if (!_fMap.ContainsKey(f))
        {
            throw new NoSuchObjectException();
        }
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        _conneF.Clear();
        _curFace = _fMap.GetValue(f).RootFace();

        for (int i = 0; i < _fMap.Count; i++)
        {
            Face theF = ShapeCasts.Face(_fMap.KeyAt(i));
            if (_fMap.GetValue(theF).RootFace().IsSame(_curFace))
            {
                _conneF.Append(theF);
            }
        }

        return _conneF;
    }

    //=================================================================================================

    /// <summary>The faces on which a modification has been given.</summary>
    /// <exception cref="NotDoneException">An Add failed.</exception>
    public NCollection_List<Shape> ModifiedFaces()
    {
        if (!_badShape.IsNull)
        {
            throw new NotDoneException();
        }
        _conneF.Clear();

        for (int i = 0; i < _fMap.Count; i++)
        {
            Face theF = ShapeCasts.Face(_fMap.KeyAt(i));
            if (!_fMap.GetValue(theF).RootFace().IsNull)
            {
                _conneF.Append(theF);
            }
        }

        return _conneF;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        s = null;
        l = new Placement();
        tol = 0.0;
        revWires = false;
        revFace = false;
        if (!IsDone)
        {
            throw new DomainErrorException();
        }

        if (!_fMap.ContainsKey(f) || !_fMap.GetValue(f).NewGeometry())
        {
            return false;
        }

        revWires = false;
        revFace = false;
        tol = Tool.Tolerance(f);

        s = Tool.Surface(f, out l);

        l = new Placement(); // L.Identity()

        s = _fMap.GetValue(f).Geometry();

        return true;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement l, out double tol)
    {
        c = null;
        l = new Placement();
        tol = 0.0;
        if (!IsDone)
        {
            throw new DomainErrorException();
        }

        if (!_eMap.ContainsKey(e))
        {
            return false;
        }

        DraftEdgeInfo einf = _eMap.GetValue(e);
        if (!_eMap.GetValue(e).NewGeometry())
        {
            return false;
        }

        tol = einf.Tolerance();
        tol = Max(tol, Tool.Tolerance(e));
        l = new Placement(); // L.Identity()
        c = _eMap.GetValue(e).Geometry();

        return true;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        p = new Point3D();
        tol = 0.0;
        if (!IsDone)
        {
            throw new DomainErrorException();
        }

        if (!_vMap.ContainsKey(v))
        {
            return false;
        }

        tol = Tool.Tolerance(v);
        p = _vMap.GetValue(v).Geometry();
        return true;
    }

    //=================================================================================================

    /// <remarks>Writes the range of <paramref name="newE"/>, the modifier's new edge.</remarks>
    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewCurve2d(in Edge e,
                                    in Face f,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? c,
                                    out double tol)
    {
        c = null;
        tol = 0.0;
        if (!IsDone)
        {
            throw new DomainErrorException();
        }

        if (!_eMap.ContainsKey(e))
        {
            return false;
        }

        Tool.Range(newE, out double fp, out double lp);

        Surface sb = _fMap.GetValue(f).Geometry()!;

        tol = Tool.Tolerance(e);

        DraftEdgeInfo einf = _eMap.GetValue(e);
        if (einf.FirstFace().IsSame(f) && einf.FirstPC() is not null)
        {
            c = einf.FirstPC();
        }
        else if (einf.SecondFace().IsSame(f) && einf.SecondPC() is not null)
        {
            c = einf.SecondPC();
        }
        else
        {
            if (!_eMap.GetValue(e).NewGeometry())
            {
                Tool.Range(e, out double fpi, out double lpi);
                if (fpi <= fp && fp <= lpi && fpi <= lp && lp <= lpi)
                {
                    return false;
                }
            }

            //  if (!BRep_Tool::IsClosed(E,F)) {
            Tool.Range(newE, out fp, out lp);
            TrimmedCurve3D tc = new(_eMap.GetValue(e).Geometry()!, fp, lp);
            fp = tc.FirstParameter();
            lp = tc.LastParameter();
            BrepBuilder b = new();
            b.Range(newE, fp, lp);
            c = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(tc, fp, lp, sb, ref tol);
        }

        Type typs = sb.GetType();
        if (typs == typeof(TrimmedSurface))
        {
            sb = ((TrimmedSurface)sb).BasisSurface();
            typs = sb.GetType();
        }

        bool jeRecadre = false;
        if (typs == typeof(ExtrusionSurface))
        {
            Curve3D aC = ((ExtrusionSurface)sb).BasisCurve();
            Type typc = aC.GetType();
            if (typc == typeof(CircleCurve3D))
            {
                jeRecadre = true;
            }
        }

        jeRecadre = jeRecadre || (typs == typeof(CylindricalSurface))
                    || (typs == typeof(SphericalSurface))
                    || (typs == typeof(ConicalSurface));

        if (jeRecadre)
        {
            bool bTranslate;
            double d2;
            Point2D pf, newPF;
            Vector2D v2DT = new(), vectra = new(2.0 * M_PI, 0.0);
            Curve2D c2DE;
            //
            c2DE = Tool.CurveOnSurface(e, f, out double t1, out double t2)!;
            //
            pf = c2DE.Value(0.5 * (t1 + t2));
            //
            newPF = c!.Value(0.5 * (fp + lp));
            //
            d2 = newPF.SquareDistance(pf);
            //
            bTranslate = false;
            if (newPF.Translated(vectra).SquareDistance(pf) < d2)
            {
                v2DT = vectra;
                bTranslate = !bTranslate; // True
            }
            else if (newPF.Translated(-vectra).SquareDistance(pf) < d2)
            {
                v2DT = -vectra;
                bTranslate = !bTranslate; // True
            }
            //
            if (bTranslate)
            {
                // OCCT.NET: OCCT translates the curve in place, which may be the one kept for the edge (FirstPC or
                // SecondPC); a frozen one (already given to a modifier's edge) is translated on a copy.
                if (c.IsFrozen)
                {
                    c = (Curve2D)c.Copy();
                }
                c.Translate(v2DT);
            }
        }
        //
        Curve3D? c3d = Tool.Curve(newE, out fp, out lp);
        tol = BrepTools.EvalAndUpdateTol(newE, c3d!, c!, sb, fp, lp);
        return true;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol)
    {
        if (!IsDone)
        {
            throw new DomainErrorException();
        }

        if (!_vMap.ContainsKey(v))
        {
            return false;
        }

        p = _vMap.GetValue(v).Parameter(e);
        Curve3D gc = _eMap.GetValue(e).Geometry()!;
        Type typc = gc.GetType();
        if (typc == typeof(TrimmedCurve3D))
        {
            // OCCT down-casts to Geom_TrimmedCurve and keeps the trimmed curve.
            typc = gc.GetType();
        }

        if (gc.IsClosed)
        {
            Vertex fv = Exploration.FirstVertex(e);
            double paramf;
            if (_vMap.ContainsKey(fv))
            {
                paramf = _vMap.GetValue(fv).Parameter(e);
            }
            else
            {
                paramf = Tool.Parameter(fv, e);
            }

            // Patch
            double firstPar = gc.FirstParameter(), lastPar = gc.LastParameter();
            double pconf = global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (Math.Abs(paramf - lastPar) <= pconf)
            {
                paramf = firstPar;
                fv.Orientation(e.Orientation());
                if (v.IsEqual(fv))
                {
                    p = paramf;
                }
            }

            fv.Orientation(e.Orientation());
            if (!v.IsEqual(fv) && p <= paramf)
            {
                if (gc.IsPeriodic)
                {
                    p += gc.Period();
                }
                else
                {
                    p = gc.LastParameter();
                }
            }
        }

        tol = Max(Tool.Tolerance(v), Tool.Tolerance(e));
        return true;
    }

    //=================================================================================================

    public override Continuity Continuity(in Edge e,
                                             in Face f1,
                                             in Face f2,
                                             in Edge newE,
                                             in Face newF1,
                                             in Face newF2)
    {
        return Tool.Continuity(e, f1, f2);
    }
}
