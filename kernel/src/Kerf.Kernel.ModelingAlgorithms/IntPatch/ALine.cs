// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALine.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// An intersection line described by a parametrised curve (<see cref="Curve"/>, the intersection of a
/// cylinder or a cone with a quadric), with its vertices.
/// </summary>
/// <remarks>
/// A working object (decision 609). The curve is a struct: the line keeps a copy, as OCCT does, and
/// <see cref="Curve"/> returns a copy. <see cref="Vertex"/>, <see cref="FirstPoint"/> and <see cref="LastPoint"/>
/// return copies; <see cref="ChangeVertex"/> returns a reference to the stored vertex.
/// </remarks>
internal class ALine : PatchLine
{
    private const double Pconfusion = 0.00001;

    private readonly Curve _curv;
    private bool _fipt;
    private bool _lapt;
    private int _indf;
    private int _indl;
    private readonly NCollection_Sequence<PatchPoint> _svtx = new();

    /// <summary>Creates an analytic intersection line when the transitions are in or out.</summary>
    public ALine(in Curve c, bool tang, WalkingTypeTrans trans1, WalkingTypeTrans trans2)
        : base(tang, trans1, trans2)
    {
        _typ = PatchLineType.Analytic;
        _curv = c;
    }

    /// <summary>Creates an analytic intersection line when the transitions are touch.</summary>
    public ALine(in Curve c, bool tang, WalkingSituation situ1, WalkingSituation situ2)
        : base(tang, situ1, situ2)
    {
        _typ = PatchLineType.Analytic;
        _curv = c;
    }

    /// <summary>Creates an analytic intersection line when the transitions are undecided.</summary>
    public ALine(in Curve c, bool tang)
        : base(tang)
    {
        _typ = PatchLineType.Analytic;
        _curv = c;
    }

    /// <summary>Adds a vertex at the end of the list.</summary>
    public void AddVertex(in PatchPoint vtXj) => _svtx.Append(vtXj);

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

    /// <summary>
    /// The first parameter on the line; <paramref name="isIncluded"/> if <see cref="Value"/> and <see cref="D1"/> may
    /// be called with it (otherwise only with greater parameters).
    /// </summary>
    public double FirstParameter(out bool isIncluded)
    {
        _curv.Domain(out double first, out _);
        isIncluded = !_curv.IsFirstOpen;
        return first;
    }

    /// <summary>
    /// The last parameter on the line; <paramref name="isIncluded"/> if <see cref="Value"/> and <see cref="D1"/> may
    /// be called with it (otherwise only with smaller parameters).
    /// </summary>
    public double LastParameter(out bool isIncluded)
    {
        _curv.Domain(out _, out double last);
        isIncluded = !_curv.IsLastOpen;
        return last;
    }

    /// <summary>The point of parameter <paramref name="u"/> (Standard_DomainError out of the domain).</summary>
    public Point3D Value(double u) => _curv.Value(u); // Value leve l exception DomainError

    /// <summary>
    /// The point at <paramref name="u"/>, and true with the derivative <paramref name="du"/> where it is defined.
    /// </summary>
    public bool D1(double u, out Point3D p, out Vector3D du) => _curv.D1u(u, out p, out du); // D1u leve l exception DomainError

    /// <summary>The parameters of the point <paramref name="p"/> on the curve, sorted ascending (none if it is not on it).</summary>
    public void FindParameter(in Point3D p, NCollection_List<double> theParams) => _curv.FindParameter(p, theParams);

    /// <summary>True if the line has a known first point (<see cref="FirstPoint"/>).</summary>
    public bool HasFirstPoint => _fipt;

    /// <summary>True if the line has a known last point (<see cref="LastPoint"/>).</summary>
    public bool HasLastPoint => _lapt;

    /// <summary>The first point; throws Standard_DomainError if there is none.</summary>
    public PatchPoint FirstPoint()
    {
        if (!_fipt)
        {
            throw new DomainErrorException();
        }
        return _svtx.Value(_indf);
    }

    /// <summary>The last point; throws Standard_DomainError if there is none.</summary>
    public PatchPoint LastPoint()
    {
        if (!_lapt)
        {
            throw new DomainErrorException();
        }
        return _svtx.Value(_indl);
    }

    public int VertexCount => _svtx.Length();

    /// <summary>The vertex of range <paramref name="index"/>.</summary>
    public PatchPoint Vertex(int index) => _svtx.Value(index);

    /// <summary>The vertex of range <paramref name="index"/>, to change in place.</summary>
    public ref PatchPoint ChangeVertex(int index) => ref _svtx.ChangeValue(index);

    /// <summary>The curve (a copy).</summary>
    public Curve Curve() => (_curv);

    /// <summary>
    /// Sets the parameters of all the vertices on the line: adds the other representations of a vertex on a line
    /// longer than a period, sorts them, removes the duplicates, handles closed lines, drops the end vertices on no
    /// restriction and merges the vertices of equal parameters.
    /// </summary>
    public void ComputeVertexParameters(double tol)
    {
        bool sortIsOK, aPointDeleted;
        bool sortAgain = true;
        int nbvtx, i, j;
        double paramMinOnLine, paramMaxOnLine;
        bool openFirst, openLast;

        paramMinOnLine = FirstParameter(out openFirst);
        paramMaxOnLine = LastParameter(out openLast);

        //----------------------------------------------------------
        //--     F i l t r e   s u r   r e s t r i c t i o n s   --
        //----------------------------------------------------------
        //-- deux vertex sur la meme restriction et seulement
        //-- sur celle ci ne doivent pas avoir le meme parametre
        //--
        //--

        //-- Le tri est necessaire si suppression du first ou du last point
        nbvtx = VertexCount;

        //-- On verifie qu un vertex a bien toute ses representations :
        //-- Cas tres rare : point de tangence sur un debut de ligne
        //-- et la ligne fait 2 * 2 PI de parametrage.

        for (i = 1; i <= nbvtx; i++)
        {
            // C++ holds a reference; the vertex is only read while the sequence grows, so a copy is the same.
            PatchPoint vtx = _svtx.Value(i);
            double p = vtx.ParameterOnLine();
            double pmpimpi = p - M_PI - M_PI;
            if (pmpimpi >= paramMinOnLine)
            {
                Point3D p1 = Value(pmpimpi);
                double d1 = p1.DistanceTo(vtx.Value());
                if (d1 < tol)
                {
                    PatchPoint ovtx = vtx;
                    ovtx.SetParameter(pmpimpi);
                    _svtx.Append(ovtx);
                }
            }
            pmpimpi = p + M_PI + M_PI;
            if (pmpimpi <= paramMaxOnLine)
            {
                Point3D p1 = Value(pmpimpi);
                double d1 = p1.DistanceTo(vtx.Value());
                if (d1 < tol)
                {
                    PatchPoint ovtx = vtx;
                    ovtx.SetParameter(pmpimpi);
                    _svtx.Append(ovtx);
                }
            }
        }

        nbvtx = VertexCount;

        if (nbvtx <= 0)
        {
            return;
        }
        do
        {
            sortIsOK = true;
            for (i = 2; i <= nbvtx; i++)
            {
                if (_svtx.Value(i - 1).ParameterOnLine() > _svtx.Value(i).ParameterOnLine())
                {
                    sortIsOK = false;
                    _svtx.Exchange(i - 1, i);
                    ExchangeIndices(i);
                }
            }
        } while (!sortIsOK);

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
                                if (Math.Abs(vtXi.ParameterOnArc1() - vtXj.ParameterOnArc1()) <= Pconfusion)
                                {
                                    if (Math.Abs(vtXi.ParameterOnLine() - vtXj.ParameterOnLine()) <= Pconfusion)
                                    {
                                        if (ReferenceEquals(vtXi.ArcOnS1(), vtXj.ArcOnS1()))
                                        {
                                            RemoveDuplicate(j, ref nbvtx);
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
                                if (Math.Abs(vtXi.ParameterOnArc2() - vtXj.ParameterOnArc2()) <= Pconfusion)
                                {
                                    if (Math.Abs(vtXi.ParameterOnLine() - vtXj.ParameterOnLine()) <= Pconfusion)
                                    {
                                        if (ReferenceEquals(vtXi.ArcOnS2(), vtXj.ArcOnS2()))
                                        {
                                            RemoveDuplicate(j, ref nbvtx);
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

        //----------------------------------------------------------
        //-- Tri des vertex et suppression des Vtx superflus
        //--
        do
        {
            nbvtx = VertexCount;
            if (sortAgain)
            {
                do
                {
                    sortIsOK = true;
                    for (i = 2; i <= nbvtx; i++)
                    {
                        if (_svtx.Value(i - 1).ParameterOnLine() > _svtx.Value(i).ParameterOnLine())
                        {
                            sortIsOK = false;
                            _svtx.Exchange(i - 1, i);
                            ExchangeIndices(i);
                        }
                    }
                } while (!sortIsOK);
            }
            sortAgain = false;
            sortIsOK = true;
            for (i = 2; i <= nbvtx && sortIsOK; i++)
            {
                PatchPoint vtx = _svtx.Value(i);
                for (j = 1; j < i && sortIsOK; j++)
                {
                    PatchPoint vtxm1 = _svtx.Value(j);
                    bool kill = false;
                    bool killm1 = false;
                    if (Math.Abs(vtxm1.ParameterOnLine() - vtx.ParameterOnLine()) < Pconfusion)
                    {
                        if (vtxm1.IsOnDomS1 && vtx.IsOnDomS1)
                        { //-- OnS1    OnS1
                            if (ReferenceEquals(vtxm1.ArcOnS1(), vtx.ArcOnS1()))
                            { //-- OnS1 == OnS1
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
                                            kill = true;
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
                        { //-- Pas OnS1  et  OnS1
                            if (!vtxm1.IsOnDomS2 && !vtx.IsOnDomS2)
                            {
                                if (vtxm1.IsOnDomS1 && !vtx.IsOnDomS1)
                                {
                                    kill = true;
                                }
                                else if (vtx.IsOnDomS1 && !vtxm1.IsOnDomS1)
                                {
                                    killm1 = true;
                                }
                            }
                        }

                        if (!(kill || killm1))
                        {
                            if (vtxm1.IsOnDomS2 && vtx.IsOnDomS2)
                            { //-- OnS2    OnS2
                                if (ReferenceEquals(vtxm1.ArcOnS2(), vtx.ArcOnS2()))
                                { //-- OnS2 == OnS2
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
                                                kill = true;
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
                            { //-- Pas OnS2  et  OnS2
                                if (!vtxm1.IsOnDomS1 && !vtx.IsOnDomS1)
                                {
                                    if (vtxm1.IsOnDomS2 && !vtx.IsOnDomS2)
                                    {
                                        kill = true;
                                    }
                                    else if (vtx.IsOnDomS2 && !vtxm1.IsOnDomS2)
                                    {
                                        killm1 = true;
                                    }
                                }
                            }
                        }
                        //-- On a j < i
                        if (kill)
                        {
                            sortIsOK = false;
                            if (_lapt)
                            {
                                if (_indl > i)
                                {
                                    _indl--;
                                }
                                else if (_indl == i)
                                {
                                    _indl = j;
                                }
                            }
                            if (_fipt)
                            {
                                if (_indf > i)
                                {
                                    _indf--;
                                }
                                else if (_indf == i)
                                {
                                    _indf = j;
                                }
                            }
                            _svtx.Remove(i);
                            nbvtx--;
                        }
                        else if (killm1)
                        {
                            sortIsOK = false;
                            if (_lapt)
                            {
                                if (_indl > j)
                                {
                                    _indl--;
                                }
                                else if (_indl == j)
                                {
                                    _indl = i - 1;
                                }
                            }
                            if (_fipt)
                            {
                                if (_indf > j)
                                {
                                    _indf--;
                                }
                                else if (_indf == j)
                                {
                                    _indf = i - 1;
                                }
                            }
                            _svtx.Remove(j);
                            nbvtx--;
                        }
                    }
                }
            }
        } while (!sortIsOK);

        //----------------------------------------------------------
        //--   Traitement des lignes periodiques                  --
        //----------------------------------------------------------
        if (!openFirst && !openLast)
        {
            nbvtx = VertexCount;

            PatchPoint vtx0 = _svtx.Value(1);
            PatchPoint vtxn = _svtx.Value(nbvtx);
            if (vtx0.ParameterOnLine() == paramMinOnLine)
            {
                if (vtxn.ParameterOnLine() != paramMaxOnLine)
                {
                    Point3D pn = Value(paramMaxOnLine);
                    double d = pn.DistanceTo(vtx0.Value());
                    if (d <= tol)
                    {
                        PatchPoint ovtx = vtx0;
                        ovtx.SetParameter(paramMaxOnLine);
                        _svtx.Append(ovtx);
                    }
                }
                else
                {
                    // OCCT: unreachable (VTX0 is at ParamMinOnLine in this branch), so a closed line whose last
                    // vertex is at its end never gets the vertex at its start (see occt-issues.md)
                    if (vtxn.ParameterOnLine() == paramMaxOnLine)
                    {
                        if (vtx0.ParameterOnLine() != paramMinOnLine)
                        {
                            Point3D p0 = Value(paramMinOnLine);
                            double d = p0.DistanceTo(vtx0.Value());
                            if (d <= tol)
                            {
                                PatchPoint ovtx = vtxn;
                                ovtx.SetParameter(paramMinOnLine);
                                _svtx.Prepend(ovtx);
                            }
                        }
                    }
                }
            }
        }
        //---------------------------------------------------------
        //-- Faut il supprimer le premier et le dernier point
        //--
        nbvtx = VertexCount;
        if (nbvtx > 1)
        {
            PatchPoint vtx0 = _svtx.Value(1);
            if ((!vtx0.IsOnDomS1) && (!vtx0.IsOnDomS2))
            {
                _svtx.Remove(1);
                nbvtx--;
                if (_lapt)
                {
                    _indl--;
                }
            }
        }
        if (nbvtx > 1)
        {
            PatchPoint vtx0 = _svtx.Value(nbvtx);
            if ((!vtx0.IsOnDomS1) && (!vtx0.IsOnDomS2))
            {
                _svtx.Remove(nbvtx);
                if (_lapt)
                {
                    _indl--;
                }
            }
        }

        //-- Si 2 vertex ont le meme parametre   on identifie le p3d
        nbvtx = VertexCount;
        do
        {
            sortIsOK = true;
            for (i = 2; i <= nbvtx; i++)
            {
                ref PatchPoint vtx = ref _svtx.ChangeValue(i);
                ref PatchPoint vtXm1 = ref _svtx.ChangeValue(i - 1);
                if (Math.Abs(vtx.ParameterOnLine() - vtXm1.ParameterOnLine()) < Pconfusion)
                {
                    if (vtx.IsOnDomS1 && !vtXm1.IsOnDomS1)
                    {
                        vtXm1.SetArc(true, vtx.ArcOnS1(), vtx.ParameterOnArc1(), vtx.TransitionLineArc1(),
                                     vtx.TransitionOnS1());
                    }
                    else if (vtXm1.IsOnDomS1 && !vtx.IsOnDomS1)
                    {
                        vtx.SetArc(true, vtXm1.ArcOnS1(), vtXm1.ParameterOnArc1(), vtXm1.TransitionLineArc1(),
                                   vtXm1.TransitionOnS1());
                    }
                    if (vtx.IsVertexOnS1 && !vtXm1.IsVertexOnS1)
                    {
                        vtXm1.SetVertex(true, vtx.VertexOnS1());
                        vtXm1.SetArc(true, vtx.ArcOnS1(), vtx.ParameterOnArc1(), vtx.TransitionLineArc1(),
                                     vtx.TransitionOnS1());
                    }
                    else if (vtXm1.IsVertexOnS1 && !vtx.IsVertexOnS1)
                    {
                        vtx.SetVertex(true, vtXm1.VertexOnS1());
                        vtx.SetArc(true, vtXm1.ArcOnS1(), vtXm1.ParameterOnArc1(), vtXm1.TransitionLineArc1(),
                                   vtXm1.TransitionOnS1());
                    }

                    if (vtx.IsOnDomS2 && !vtXm1.IsOnDomS2)
                    {
                        vtXm1.SetArc(false, vtx.ArcOnS2(), vtx.ParameterOnArc2(), vtx.TransitionLineArc2(),
                                     vtx.TransitionOnS2());
                    }
                    else if (vtXm1.IsOnDomS2 && !vtx.IsOnDomS2)
                    {
                        vtx.SetArc(false, vtXm1.ArcOnS2(), vtXm1.ParameterOnArc2(), vtXm1.TransitionLineArc2(),
                                   vtXm1.TransitionOnS2());
                    }
                    if (vtx.IsVertexOnS2 && !vtXm1.IsVertexOnS2)
                    {
                        vtXm1.SetVertex(false, vtx.VertexOnS2());
                        vtXm1.SetArc(false, vtx.ArcOnS2(), vtx.ParameterOnArc2(), vtx.TransitionLineArc2(),
                                     vtx.TransitionOnS2());
                    }
                    else if (vtXm1.IsVertexOnS2 && !vtx.IsVertexOnS2)
                    {
                        vtx.SetVertex(false, vtXm1.VertexOnS2());
                        vtx.SetArc(false, vtXm1.ArcOnS2(), vtXm1.ParameterOnArc2(), vtXm1.TransitionLineArc2(),
                                   vtXm1.TransitionOnS2());
                    }

                    if (vtx.Value().SquareDistanceTo(vtXm1.Value()) > 1e-12)
                    {
                        PatchPoint copyVtx = vtXm1;
                        vtXm1.SetParameter(vtx.ParameterOnLine());
                        vtXm1.SetValue(vtx.Value(), vtx.Tolerance(), vtx.IsTangencyPoint);
                        vtx.Parameters(out double u1, out double v1, out double u2, out double v2);
                        vtXm1.SetParameters(u1, v1, u2, v2);
                        if (copyVtx.IsOnDomS1)
                        {
                            vtXm1.SetArc(true, copyVtx.ArcOnS1(), copyVtx.ParameterOnArc1(),
                                         copyVtx.TransitionLineArc1(), copyVtx.TransitionOnS1());
                        }
                        if (copyVtx.IsOnDomS2)
                        {
                            vtXm1.SetArc(false, copyVtx.ArcOnS2(), copyVtx.ParameterOnArc2(),
                                         copyVtx.TransitionLineArc2(), copyVtx.TransitionOnS2());
                        }
                        if (copyVtx.IsVertexOnS1)
                        {
                            vtXm1.SetVertex(true, copyVtx.VertexOnS1());
                            vtXm1.SetArc(true, copyVtx.ArcOnS1(), copyVtx.ParameterOnArc1(),
                                         copyVtx.TransitionLineArc1(), copyVtx.TransitionOnS1());
                        }
                        if (copyVtx.IsVertexOnS2)
                        {
                            vtXm1.SetVertex(false, copyVtx.VertexOnS2());
                            vtXm1.SetArc(false, copyVtx.ArcOnS2(), copyVtx.ParameterOnArc2(),
                                         copyVtx.TransitionLineArc2(), copyVtx.TransitionOnS2());
                        }

                        sortIsOK = false;
                        //-- std::cout<<" IntPatch_ALine : ComputeVertexParameters : Ajust "<<std::endl;
                    }
                }
            }
        } while (!sortIsOK);
    }

    // The index updates of an exchange of the vertices i - 1 and i, that OCCT writes out in both sorts.
    private void ExchangeIndices(int i)
    {
        if (_fipt)
        {
            if (_indf == i)
            {
                _indf = i - 1;
            }
            else if (_indf == i - 1)
            {
                _indf = i;
            }
        }
        if (_lapt)
        {
            if (_indl == i)
            {
                _indl = i - 1;
            }
            else if (_indl == i - 1)
            {
                _indl = i;
            }
        }
    }

    // The removal of a duplicate on a restriction (OCCT's ">=" index updates), written out twice in OCCT.
    private void RemoveDuplicate(int j, ref int nbvtx)
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
    }
}
