// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_RLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_RLine.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_RLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// An intersection line that is a restriction (an arc) of one of the surfaces, with its vertices and, optionally, a
/// polygon of points (<see cref="Curve"/>).
/// </summary>
/// <remarks>
/// A working object (decision 609). OCCT declares <c>SetParamOnS1</c> and <c>SetParamOnS2</c> but never defines
/// them: left out. <see cref="Dump(int, TextWriter)"/> is OCCT's <c>Dump</c> (to stdout).
/// </remarks>
public class RLine : PointLine
{
    private Curve2DAdaptorBase? _arcOnS1;
    private Curve2DAdaptorBase? _arcOnS2;
    private bool _onS1;
    private bool _onS2;
    // ParamInf1, ParamSup1, ParamInf2, ParamSup2: initialised to 0 by OCCT and never read: left out (decision 143).
    private LineOn2S? _curv;
    private bool _fipt;
    private bool _lapt;
    private int _indf;
    private int _indl;
    private readonly NCollection_Sequence<PatchPoint> _svtx = new();

    /// <summary>A restriction line whose transitions are in or out.</summary>
    public RLine(bool tang, WalkingTypeTrans trans1, WalkingTypeTrans trans2)
        : base(tang, trans1, trans2)
    {
        _fipt = false;
        _lapt = false;
        _indf = 0;
        _indl = 0;
        _typ = PatchLineType.Restriction;
        _onS2 = false;
        _onS1 = false;
    }

    /// <summary>A restriction line whose transitions are touch.</summary>
    public RLine(bool tang, WalkingSituation situ1, WalkingSituation situ2)
        : base(tang, situ1, situ2)
    {
        _fipt = false;
        _lapt = false;
        _indf = 0;
        _indl = 0;
        _typ = PatchLineType.Restriction;
        _onS2 = false;
        _onS1 = false;
    }

    /// <summary>A restriction line whose transitions are undecided.</summary>
    public RLine(bool tang)
        : base(tang)
    {
        _fipt = false;
        _lapt = false;
        _indf = 0;
        _indl = 0;
        _typ = PatchLineType.Restriction;
        _onS2 = false;
        _onS1 = false;
    }

    /// <summary>
    /// A copy of <paramref name="other"/>, as OCCT's implicit copy constructor: the vertices are copied, the arcs
    /// and the points (<see cref="Curve"/>) are shared.
    /// </summary>
    public RLine(RLine other)
        : base(other)
    {
        _arcOnS1 = other._arcOnS1;
        _arcOnS2 = other._arcOnS2;
        _onS1 = other._onS1;
        _onS2 = other._onS2;
        _curv = other._curv;
        _fipt = other._fipt;
        _lapt = other._lapt;
        _indf = other._indf;
        _indl = other._indl;
        for (int i = 1; i <= other._svtx.Length(); i++)
        {
            _svtx.Append(other._svtx.Value(i));
        }
    }

    public override void AddVertex(in PatchPoint pnt, bool isPrepend = false)
    {
        if (isPrepend)
        {
            _svtx.Prepend(pnt);
        }
        else
        {
            _svtx.Append(pnt);
        }
    }

    /// <summary>Replaces the vertex of range <paramref name="index"/>.</summary>
    public void Replace(int index, in PatchPoint pnt) => _svtx[index] = pnt;

    public void SetFirstPoint(int indFirst)
    {
        _fipt = true;
        _indf = indFirst;
    }

    public void SetLastPoint(int indLast)
    {
        _lapt = true;
        _indl = indLast;
    }

    /// <summary>Sets the polygon of points of the line (kept, not copied, as OCCT's handle).</summary>
    public void Add(LineOn2S l) => _curv = l;

    /// <summary>True if the line is an arc of restriction of the first surface.</summary>
    public bool IsArcOnS1 => _onS1;

    /// <summary>True if the line is an arc of restriction of the second surface.</summary>
    public bool IsArcOnS2 => _onS2;

    public void SetArcOnS1(Curve2DAdaptorBase a)
    {
        _arcOnS1 = a;
        _onS1 = true;
    }

    public void SetArcOnS2(Curve2DAdaptorBase a)
    {
        _arcOnS2 = a;
        _onS2 = true;
    }

    public Curve2DAdaptorBase? ArcOnS1() => _arcOnS1;

    public Curve2DAdaptorBase? ArcOnS2() => _arcOnS2;

    /// <summary>
    /// The smallest and largest parameters on the line of the vertices if the line is on the first surface's arc;
    /// otherwise zeros.
    /// </summary>
    public void ParamOnS1(out double a, out double b)
    {
        if (_onS1)
        {
            a = RealLast();
            b = -a;
            for (int i = _svtx.Length(); i >= 1; i--)
            {
                double p = _svtx.Value(i).ParameterOnLine();
                if (p < a)
                {
                    a = p;
                }
                if (p > b)
                {
                    b = p;
                }
            }
        }
        else
        {
            a = b = 0.0;
        }
    }

    /// <summary>As <see cref="ParamOnS1"/>, for the second surface's arc.</summary>
    public void ParamOnS2(out double a, out double b)
    {
        if (_onS2)
        {
            a = RealLast();
            b = -a;
            for (int i = _svtx.Length(); i >= 1; i--)
            {
                double p = _svtx.Value(i).ParameterOnLine();
                if (p < a)
                {
                    a = p;
                }
                if (p > b)
                {
                    b = p;
                }
            }
        }
        else
        {
            a = b = 0.0;
        }
    }

    public bool HasFirstPoint => _fipt;

    public bool HasLastPoint => _lapt;

    /// <summary>The vertex that is the first point; throws Standard_DomainError if there is none.</summary>
    public PatchPoint FirstPoint()
    {
        if (!_fipt)
        {
            throw new DomainErrorException();
        }
        return _svtx.Value(_indf);
    }

    /// <summary>The vertex that is the last point; throws Standard_DomainError if there is none.</summary>
    public PatchPoint LastPoint()
    {
        if (!_lapt)
        {
            throw new DomainErrorException();
        }
        return _svtx.Value(_indl);
    }

    public override int VertexCount => _svtx.Length();

    public override PatchPoint Vertex(int index) => _svtx.Value(index);

    public override ref PatchPoint ChangeVertex(int index) => ref _svtx[index];

    public override void RemoveVertex(int index)
    {
        if ((index < 1) || (index > VertexCount))
        {
            throw new OutOfRangeException("Cannot delete not existing vertex");
        }
        _svtx.Remove(index);
    }

    /// <summary>True if the line has a polygon of points.</summary>
    public bool HasPolygon => (_curv is not null);

    /// <summary>The number of points of the polygon; throws Standard_DomainError if there is none.</summary>
    public override int NbPnts()
    {
        if (_curv is null)
        {
            throw new DomainErrorException();
        }
        return _curv.PointCount;
    }

    /// <summary>The point of range <paramref name="index"/> of the polygon; throws Standard_DomainError if there is none.</summary>
    public override PntOn2S Point(int index)
    {
        if (_curv is null)
        {
            throw new DomainErrorException();
        }
        return _curv.Value(index);
    }

    /// <summary>Sets the point of range <paramref name="index"/> of the polygon to the vertex's point and parameters.</summary>
    public void SetPoint(int index, in PatchPoint thepoint) => _curv!.Value(index, thepoint.PntOn2S());

    public override LineOn2S? Curve() => (_curv);

    public override bool IsOutSurf1Box(in Point2D p) => _curv!.IsOutSurf1Box(p);

    public override bool IsOutSurf2Box(in Point2D p) => _curv!.IsOutSurf2Box(p);

    public override bool IsOutBox(in Point3D p) => _curv!.IsOutBox(p);

    public override void ClearVertexes() => _svtx.Clear();

    /// <summary>Replaces the polygon of points.</summary>
    public void SetCurve(LineOn2S newCurve) => _curv = newCurve;

    /// <summary>
    /// Removes the duplicate vertices (same arc and parameter, or the vertex on fewer arcs at the same parameter), sorts
    /// them by parameter on the line, and makes the first and the last the line's first and last points. The
    /// tolerance is not used (as in OCCT).
    /// </summary>
    public void ComputeVertexParameters(double tol)
    {
        int i, j, nbvtx; // k;

        bool aPointDeleted = false;
        //----------------------------------------------------------
        //--     F i l t r e   s u r   r e s t r i c t i o n s   --
        //----------------------------------------------------------
        //-- deux vertex sur la meme restriction et seulement
        //-- sur celle ci ne doivent pas avoir le meme parametre
        //--
        nbvtx = VertexCount;
        do
        {
            aPointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!aPointDeleted); i++)
            {
                PatchPoint vtXi = _svtx.Value(i);
                if ((vtXi.IsOnDomS1) && (!vtXi.IsOnDomS2))
                {
                    for (j = 1; (j <= nbvtx) && (!aPointDeleted); j++)
                    {
                        if (i != j)
                        {
                            PatchPoint vtXj = _svtx.Value(j);
                            if ((vtXj.IsOnDomS1) && (!vtXj.IsOnDomS2))
                            {
                                if (vtXi.ParameterOnLine() == vtXj.ParameterOnLine())
                                {
                                    if (ReferenceEquals(vtXi.ArcOnS1(), vtXj.ArcOnS1()))
                                    {
                                        if (vtXi.ParameterOnArc1() == vtXj.ParameterOnArc1())
                                        {
                                            _svtx.Remove(j);
                                            nbvtx--;
                                            if (_lapt)
                                            {
                                                if (_indl >= j)
                                                {
                                                    _indl--;
                                                }
                                            }
                                            if (_fipt)
                                            {
                                                if (_indf >= j)
                                                {
                                                    _indf--;
                                                }
                                            }
                                            aPointDeleted = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } while (aPointDeleted);

        do
        {
            aPointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!aPointDeleted); i++)
            {
                PatchPoint vtXi = _svtx.Value(i);
                if ((vtXi.IsOnDomS2) && (!vtXi.IsOnDomS1))
                {
                    for (j = 1; (j <= nbvtx) && (!aPointDeleted); j++)
                    {
                        if (i != j)
                        {
                            PatchPoint vtXj = _svtx.Value(j);
                            if ((vtXj.IsOnDomS2) && (!vtXj.IsOnDomS1))
                            {
                                if (vtXi.ParameterOnLine() == vtXj.ParameterOnLine())
                                {
                                    if (ReferenceEquals(vtXi.ArcOnS2(), vtXj.ArcOnS2()))
                                    {
                                        if (vtXi.ParameterOnArc2() == vtXj.ParameterOnArc2())
                                        {
                                            _svtx.Remove(j);
                                            nbvtx--;
                                            if (_lapt)
                                            {
                                                if (_indl >= j)
                                                {
                                                    _indl--;
                                                }
                                            }
                                            if (_fipt)
                                            {
                                                if (_indf >= j)
                                                {
                                                    _indf--;
                                                }
                                            }
                                            aPointDeleted = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } while (aPointDeleted);

        nbvtx = VertexCount;

        //----------------------------------------------------
        //-- On trie les Vertex
        bool sortIsOK;
        do
        {
            sortIsOK = true;
            for (i = 2; i <= nbvtx; i++)
            {
                if (_svtx.Value(i - 1).ParameterOnLine() > _svtx.Value(i).ParameterOnLine())
                {
                    sortIsOK = false;
                    _svtx.Exchange(i - 1, i);
                }
            }
        } while (!sortIsOK);

        do
        {
            aPointDeleted = false;
            bool restrdiff;
            for (i = 1; i <= nbvtx && (!aPointDeleted); i++)
            {
                PatchPoint vtx = _svtx.Value(i);
                for (j = 1; j <= nbvtx && (!aPointDeleted); j++)
                {
                    if (i != j)
                    {
                        PatchPoint vtxm1 = _svtx.Value(j);

                        bool kill = false;
                        bool killm1 = false;
                        if (vtxm1.ParameterOnLine() == vtx.ParameterOnLine())
                        {
                            restrdiff = false;
                            if (vtxm1.IsOnDomS1 && vtx.IsOnDomS1)
                            { //-- OnS1    OnS1
                                if (ReferenceEquals(vtxm1.ArcOnS1(), vtx.ArcOnS1()))
                                { //-- OnS1 == OnS1
                                    if (vtx.ParameterOnArc1() == vtxm1.ParameterOnArc1())
                                    {
                                        if (vtxm1.IsOnDomS2)
                                        { //-- OnS1 == OnS1  OnS2
                                            if (!vtx.IsOnDomS2)
                                            { //-- OnS1 == OnS1  OnS2 PasOnS2
                                                kill = true;
                                            }
                                            else
                                            {
                                                if (ReferenceEquals(vtxm1.ArcOnS2(), vtx.ArcOnS2()))
                                                { //-- OnS1 == OnS1  OnS2 == OnS2
                                                    if (vtx.ParameterOnArc2() == vtxm1.ParameterOnArc2())
                                                    {
                                                        kill = true;
                                                    }
                                                }
                                            }
                                        }
                                        else
                                        { //-- OnS1 == OnS1  PasOnS2
                                            if (vtx.IsOnDomS2)
                                            { //-- OnS1 == OnS1  PasOnS2  OnS2
                                                killm1 = true;
                                            }
                                        }
                                    }
                                }
                                else
                                {
                                    restrdiff = true;
                                }
                            }

                            if ((!restrdiff) && (!(kill || killm1)))
                            {
                                if (vtxm1.IsOnDomS2 && vtx.IsOnDomS2)
                                { //-- OnS2    OnS2
                                    if (ReferenceEquals(vtxm1.ArcOnS2(), vtx.ArcOnS2()))
                                    { //-- OnS2 == OnS2
                                        if (vtx.ParameterOnArc2() == vtxm1.ParameterOnArc2())
                                        {
                                            if (vtxm1.IsOnDomS1)
                                            { //-- OnS2 == OnS2  OnS1
                                                if (!vtx.IsOnDomS1)
                                                { //-- OnS2 == OnS2  OnS1 PasOnS1
                                                    kill = true;
                                                }
                                                else
                                                {
                                                    if (ReferenceEquals(vtxm1.ArcOnS1(), vtx.ArcOnS1()))
                                                    { //-- OnS2 == OnS2  OnS1 == OnS1
                                                        if (vtx.ParameterOnArc1() == vtxm1.ParameterOnArc1())
                                                        {
                                                            kill = true;
                                                        }
                                                    }
                                                }
                                            }
                                            else
                                            { //-- OnS2 == OnS2  PasOnS1
                                                if (vtx.IsOnDomS1)
                                                { //-- OnS2 == OnS2  PasOnS1  OnS1
                                                    killm1 = true;
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        restrdiff = true;
                                    }
                                }
                            }
                            if (!restrdiff)
                            {
                                if (kill)
                                {
                                    aPointDeleted = true;
                                    _svtx.Remove(i);
                                    nbvtx--;
                                }
                                else if (killm1)
                                {
                                    aPointDeleted = true;
                                    _svtx.Remove(j);
                                    nbvtx--;
                                }
                            }
                        }
                    }
                }
            }
        } while (aPointDeleted);

        do
        {
            sortIsOK = true;
            for (i = 2; i <= nbvtx && sortIsOK; i++)
            {
                PatchPoint pim1 = _svtx.Value(i - 1);
                PatchPoint pii = _svtx.Value(i);
                if (pim1.ParameterOnLine() == pii.ParameterOnLine())
                {
                    if ((!pii.IsOnDomS1) && (!pii.IsOnDomS2))
                    {
                        sortIsOK = false;
                        _svtx.Remove(i);
                        nbvtx--;
                    }
                    else
                    {
                        if ((!pim1.IsOnDomS1) && (!pim1.IsOnDomS2))
                        {
                            sortIsOK = false;
                            _svtx.Remove(i - 1);
                            nbvtx--;
                        }
                    }
                }
            }
        } while (!sortIsOK);

        //----------------------------------------------------
        //-- On trie les Vertex ( Cas Bizarre )
        nbvtx = VertexCount;
        do
        {
            sortIsOK = true;
            for (i = 2; i <= nbvtx; i++)
            {
                if (_svtx.Value(i - 1).ParameterOnLine() > _svtx.Value(i).ParameterOnLine())
                {
                    sortIsOK = false;
                    _svtx.Exchange(i - 1, i);
                }
            }
        } while (!sortIsOK);

        SetFirstPoint(1);
        SetLastPoint(nbvtx);
    }

    /// <summary>
    /// Writes the line as OCCT's <c>Dump</c> prints it (see <see cref="WLine.Dump(int, TextWriter)"/>);
    /// throws Standard_DomainError if the line has no polygon.
    /// </summary>
    public void Dump(int mode, TextWriter theOut) => WLine.DumpPointLine(this, " R", mode, theOut);
}
