// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLine.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using System.Globalization;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// An intersection line between two parametric patches defined by a set of points (from a walking algorithm), with
/// its vertices, its first and last points, the periods of the surfaces and the restriction arcs it lies on.
/// </summary>
/// <remarks>
/// A working object (decision 609): the intersection algorithms split, purge and edit it in place.
/// <see cref="Vertex"/> returns a copy; <see cref="ChangeVertex"/> a reference to the stored vertex (valid until the
/// next vertex is added or removed). <see cref="Dump(int, TextWriter)"/> is OCCT's <c>Dump</c> (to stdout). It is
/// the line of GeomInt's instantiations of ApproxInt's generics (<see cref="IWLine{TSelf}"/>).
/// </remarks>
internal class WLine : PointLine, IWLine<WLine>
{
    /// <summary>The ways a WLine is created.</summary>
    public enum IntPatch_WLType
    {
        WLUnknown,
        WLImpImp,
        WLImpPrm,
        WLPrmPrm,
    }

    private readonly LineOn2S _curv;
    private bool _fipt;
    private bool _lapt;
    private int _indf;
    private int _indl;
    private readonly NCollection_Sequence<PatchPoint> _svtx = new();
    private double _u1period;
    private double _v1period;
    private double _u2period;
    private double _v2period;
    private bool _hasArcOnS1;
    private Curve2DAdaptorBase? _arcOnS1;
    private bool _hasArcOnS2;
    private Curve2DAdaptorBase? _arcOnS2;
    private bool _isPurgerAllowed;
    // identifies the way of <*this> creation
    private IntPatch_WLType _creationWay;

    /// <summary>A WLine whose transitions are in or out.</summary>
    public WLine(LineOn2S line, bool tang, WalkingTypeTrans trans1, WalkingTypeTrans trans2)
        : base(tang, trans1, trans2)
    {
        _fipt = false;
        _lapt = false;
        _hasArcOnS1 = false;
        _hasArcOnS2 = false;
        _isPurgerAllowed = true;
        _creationWay = IntPatch_WLType.WLUnknown;
        _typ = PatchLineType.Walking;
        _curv = line;
        _u1period = _v1period = _u2period = _v2period = 0.0;
    }

    /// <summary>A WLine whose transitions are touch.</summary>
    public WLine(LineOn2S line, bool tang, WalkingSituation situ1, WalkingSituation situ2)
        : base(tang, situ1, situ2)
    {
        _fipt = false;
        _lapt = false;
        _hasArcOnS1 = false;
        _hasArcOnS2 = false;
        _isPurgerAllowed = true;
        _creationWay = IntPatch_WLType.WLUnknown;
        _typ = PatchLineType.Walking;
        _curv = line;
        _u1period = _v1period = _u2period = _v2period = 0.0;
    }

    /// <summary>A WLine whose transitions are undecided.</summary>
    public WLine(LineOn2S line, bool tang)
        : base(tang)
    {
        _fipt = false;
        _lapt = false;
        _hasArcOnS1 = false;
        _hasArcOnS2 = false;
        _isPurgerAllowed = true;
        _creationWay = IntPatch_WLType.WLUnknown;
        _typ = PatchLineType.Walking;
        _curv = line;
        _u1period = _v1period = _u2period = _v2period = 0.0;
    }

    /// <summary>OCCT's <c>new IntPatch_WLine(Line, Tang)</c> in ApproxInt's generics (the TheLine macro).</summary>
    static WLine IWLine<WLine>.Create(LineOn2S lin, bool tang) => new(lin, tang);

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

    /// <summary>Sets the point of range <paramref name="index"/> of the line to the vertex's point and parameters.</summary>
    public void SetPoint(int index, in PatchPoint thepoint) => _curv.Value(index, thepoint.PntOn2S());

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

    public override int NbPnts() => _curv.PointCount;

    public override PntOn2S Point(int index) => _curv.Value(index);

    /// <summary>True if the line has a known first point (<see cref="FirstPoint()"/>).</summary>
    public bool HasFirstPoint => _fipt;

    /// <summary>True if the line has a known last point (<see cref="LastPoint()"/>).</summary>
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

    /// <summary>The vertex that is the first point, and its index in the list of vertices.</summary>
    public PatchPoint FirstPoint(out int indfirst)
    {
        if (!_fipt)
        {
            throw new DomainErrorException();
        }
        indfirst = _indf;
        return _svtx.Value(_indf);
    }

    /// <summary>The vertex that is the last point, and its index in the list of vertices.</summary>
    public PatchPoint LastPoint(out int indlast)
    {
        if (!_lapt)
        {
            throw new DomainErrorException();
        }
        indlast = _indl;
        return _svtx.Value(_indl);
    }

    public override int VertexCount => _svtx.Length();

    public override PatchPoint Vertex(int index) => _svtx.Value(index);

    public override ref PatchPoint ChangeVertex(int index) => ref _svtx[index];

    public override LineOn2S Curve() => (_curv);

    public override bool IsOutSurf1Box(in Point2D p) => _curv.IsOutSurf1Box(p);

    public override bool IsOutSurf2Box(in Point2D p) => _curv.IsOutSurf2Box(p);

    public override bool IsOutBox(in Point3D p) => _curv.IsOutBox(p);

    public void SetPeriod(double pu1, double pv1, double pu2, double pv2)
    {
        _u1period = pu1;
        _v1period = pv1;
        _u2period = pu2;
        _v2period = pv2;
    }

    public double U1Period() => (_u1period);

    public double V1Period() => (_v1period);

    public double U2Period() => (_u2period);

    public double V2Period() => (_v2period);

    public void SetArcOnS1(Curve2DAdaptorBase a)
    {
        _hasArcOnS1 = true;
        _arcOnS1 = a;
    }

    public bool HasArcOnS1 => (_hasArcOnS1);

    public Curve2DAdaptorBase? GetArcOnS1() => (_arcOnS1);

    public void SetArcOnS2(Curve2DAdaptorBase a)
    {
        _hasArcOnS2 = true;
        _arcOnS2 = a;
    }

    public bool HasArcOnS2 => (_hasArcOnS2);

    public Curve2DAdaptorBase? GetArcOnS2() => (_arcOnS2);

    public override void ClearVertexes() => _svtx.Clear();

    public override void RemoveVertex(int index)
    {
        if ((index < 1) || (index > VertexCount))
        {
            throw new OutOfRangeException("Cannot delete not existing vertex");
        }
        _svtx.Remove(index);
    }

    /// <summary>Inserts a vertex before <paramref name="index"/> (at least 1), or appends it past the end.</summary>
    public void InsertVertexBefore(int index, in PatchPoint pnt)
    {
        int nbVertexes = VertexCount;
        int anIndex = Max(index, 1);
        if (anIndex > nbVertexes)
        {
            _svtx.Append(pnt);
        }
        else
        {
            // OCCT inserts at theIndex, not at the clamped anIndex (a theIndex below 1 throws).
            _svtx.InsertBefore(index, pnt);
        }
    }

    /// <summary>Allows or forbids purging this line.</summary>
    public void EnablePurging(bool isEnabled) => _isPurgerAllowed = isEnabled;

    /// <summary>True if purging this line is allowed.</summary>
    public bool IsPurgingAllowed => _isPurgerAllowed;

    /// <summary>The way this line was created.</summary>
    public IntPatch_WLType GetCreatingWay() => _creationWay;

    /// <summary>Sets the way this line was created.</summary>
    public void SetCreatingWayInfo(IntPatch_WLType algo) => _creationWay = algo;

    private static void RecadreMemePeriode(ref double u1, ref double v1, ref double u2, ref double v2, double anu1,
                                           double anv1, double anu2, double anv2, double u1Period, double v1Period,
                                           double u2Period, double v2Period)
    {
        if (u1Period != 0)
        {
            while (anu1 - u1 > 0.8 * u1Period)
            {
                u1 += u1Period;
            }
            while (u1 - anu1 > 0.8 * u1Period)
            {
                u1 -= u1Period;
            }
        }
        if (u2Period != 0)
        {
            while (anu2 - u2 > 0.8 * u2Period)
            {
                u2 += u2Period;
            }
            while (u2 - anu2 > 0.8 * u2Period)
            {
                u2 -= u2Period;
            }
        }
        if (v1Period != 0)
        {
            while (anv1 - v1 > 0.8 * v1Period)
            {
                v1 += v1Period;
            }
            while (v1 - anv1 > 0.8 * v1Period)
            {
                v1 -= v1Period;
            }
        }
        if (v2Period != 0)
        {
            while (anv2 - v2 > 0.8 * v2Period)
            {
                v2 += v2Period;
            }
            while (v2 - anv2 > 0.8 * v2Period)
            {
                v2 -= v2Period;
            }
        }
    }

    private static void RecadreMemePeriode(ref PntOn2S pOn2S, in PntOn2S refPOn2S, double up1,
                                           double vp1, double up2, double vp2)
    {
        pOn2S.Parameters(out double u1, out double v1, out double u2, out double v2);
        refPOn2S.Parameters(out double pu1, out double pv1, out double pu2, out double pv2);
        RecadreMemePeriode(ref u1, ref v1, ref u2, ref v2, pu1, pv1, pu2, pv2, up1, vp1, up2, vp2);
        pOn2S.SetValue(u1, v1, u2, v2);
    }

    private static bool CompareVertexAndPoint(in Point3D v, in Point3D p, double tol)
    {
        double sqDist = v.SquareDistanceTo(p);
        double sqTol = tol * tol;
        return (sqDist <= sqTol);
    }

    // OCCT's SameVtxRst (a global function of IntPatch_WLine.cxx): the two vertices are on the same arcs at the
    // same parameters, or on none.
    internal static bool SameVtxRst(in PatchPoint vtx1, in PatchPoint vtx2)
    {
        if (vtx1.IsOnDomS1)
        {
            if (vtx2.IsOnDomS1)
            {
                if (ReferenceEquals(vtx1.ArcOnS1(), vtx2.ArcOnS1()))
                {
                    if (vtx1.ParameterOnArc1() == vtx2.ParameterOnArc1())
                    {
                    }
                    else
                    {
                        return (false);
                    }
                }
                else
                {
                    return (false);
                }
            }
            else
            {
                return (false);
            }
        }
        else
        {
            if (vtx2.IsOnDomS1)
            {
                return (false);
            }
        }
        if (vtx1.IsOnDomS2)
        {
            if (vtx2.IsOnDomS2)
            {
                if (ReferenceEquals(vtx1.ArcOnS2(), vtx2.ArcOnS2()))
                {
                    if (vtx1.ParameterOnArc2() == vtx2.ParameterOnArc2())
                    {
                    }
                    else
                    {
                        return (false);
                    }
                }
                else
                {
                    return (false);
                }
            }
            else
            {
                return (false);
            }
        }
        else
        {
            if (vtx2.IsOnDomS2)
            {
                return (false);
            }
        }
        return (true);
    }

    private static bool CompareVerticesOnSurf(in PatchPoint vtx1, in PatchPoint vtx2, bool onFirst)
    {
        double u1, v1, u2, v2, tolU, tolV;

        if (onFirst)
        {
            vtx1.ParametersOnS1(out u1, out v1);
            vtx2.ParametersOnS1(out u2, out v2);
        }
        else
        {
            vtx1.ParametersOnS2(out u1, out v1);
            vtx2.ParametersOnS2(out u2, out v2);
        }
        tolU = global::Kerf.Engine.Foundation.Precision.PConfusion();
        tolV = global::Kerf.Engine.Foundation.Precision.PConfusion();
        return (Math.Abs(u1 - u2) <= tolU && Math.Abs(v1 - v2) <= tolV);
    }

    private static bool CompareVerticesOnS1(in PatchPoint vtx1, in PatchPoint vtx2) =>
        CompareVerticesOnSurf(vtx1, vtx2, true);

    private static bool CompareVerticesOnS2(in PatchPoint vtx1, in PatchPoint vtx2) =>
        CompareVerticesOnSurf(vtx1, vtx2, false);

    //------------------------------------------------------------------------
    //--  En Entree : Une ligne de cheminement     +    Une Liste de Vetex
    //--
    //--   LineOn2S   :       1------2-------3-------4-----5---- ----nbp
    //--
    //--   Vertex     :   a     b c          d    e      f
    //--
    //--
    //--  En Sortie
    //--
    //--                  1--2-3-4--5--------6----7--8--9--10--------
    //--
    //--  avec   a de parametre 1
    //--         b              3
    //--
    //--   etc ...
    //--
    //--
    //-- !!!!!!!!!!!!!!! On considere que deux vertex ne peuvent pas etre
    //-- !!!!!!!!!!!!!!!  a une distance inferieure a Tol
    //------------------------------------------------------------------------
    //--
    //-- On Teste si la LineOn2S contient des points confondus.
    //-- Dans ce cas, on remove ces points.
    //--
    //------------------------------------------------------------------------

    /// <summary>
    /// Sets the parameters of all the vertices on the line: a vertex already on the line gets its point's index, the
    /// others are inserted into the line; duplicate vertices and points are removed, the vertices sorted, and first
    /// and last vertices made.
    /// </summary>
    /// <remarks>
    /// OCCT holds references to two vertices of the sequence across an <c>Append</c> (valid in OCCT's linked
    /// sequence); the port indexes the sequence at every access, which is what those references read and write.
    /// </remarks>
    public void ComputeVertexParameters(double rTol)
    {
        // MSV Oct 15, 2001: use tolerance of vertex instead of RTol where
        //                   it is possible

        int i, j, k, nbvtx, nbponline;
        int indicevertexonline;
        double indicevertex;

        bool aPointDeleted = false;
        //----------------------------------------------------------
        //--     F i l t r e   s u r   r e s t r i c t i o n s   --
        //----------------------------------------------------------
        //-- deux vertex sur la meme restriction et seulement
        //-- sur celle ci ne doivent pas avoir le meme parametre
        //--
        double tol = rTol;
        nbvtx = VertexCount;
        // DEBUGV: a dump of the vertices before the computation, compiled out in OCCT: left out.

        //-- ----------------------------------------------------------------------
        //-- Traitement des aretes de couture : On duplique les points situes
        //-- sur des restrictions differentes
        //--
        //-- Phase Creation de nouveaux points sur S1
        bool encoreunefois;
        do
        {
            nbvtx = VertexCount;
            encoreunefois = false;
            for (i = 1; i <= nbvtx && !encoreunefois; i++)
            {
                for (j = 1; j <= nbvtx && !encoreunefois; j++)
                {
                    if (i != j)
                    {
                        if (_svtx[i].ParameterOnLine() != _svtx[j].ParameterOnLine())
                        {
                            double d = _svtx[i].Value().DistanceTo(_svtx[j].Value());
                            double toli = _svtx[i].Tolerance();
                            double tolj = _svtx[j].Tolerance();
                            double maxtol = Max(toli, tolj);
                            // MSV Oct 30, 2001: compare in 2D space also;
                            //                   increase tolerances
                            if (d < maxtol || CompareVerticesOnS1(_svtx[i], _svtx[j]) || CompareVerticesOnS2(_svtx[i], _svtx[j]))
                            {
                                //-- Creation Vtx (REF:S1(i)  S2(j))    (On Garde S1(i))
                                double newtoli = Max(toli, tolj + d * 1.01);
                                double newtolj = Max(tolj, toli + d * 1.01);
                                bool acreer = false;
                                if (_svtx[i].IsOnDomS1)
                                {
                                    if (_svtx[j].IsOnDomS1)
                                    {
                                        if (!ReferenceEquals(_svtx[j].ArcOnS1(), _svtx[i].ArcOnS1()))
                                        {
                                            acreer = true;
                                        }
                                    }
                                    else
                                    {
                                        acreer = true;
                                    }
                                }
                                if (acreer)
                                {
                                    PatchPoint vtx = _svtx[j];
                                    vtx.SetArc(true, _svtx[i].ArcOnS1(), _svtx[i].ParameterOnArc1(),
                                               _svtx[i].TransitionLineArc1(), _svtx[i].TransitionOnS1());
                                    for (k = 1; !encoreunefois && k <= nbvtx; k++)
                                    {
                                        if (SameVtxRst(_svtx[k], vtx))
                                        {
                                            encoreunefois = true;
                                        }
                                    }
                                    if (!encoreunefois)
                                    {
                                        _svtx[i].SetTolerance(newtoli);
                                        _svtx[j].SetTolerance(newtolj);
                                        vtx.SetTolerance(newtolj);
                                        _svtx.Append(vtx);
                                        encoreunefois = true;
                                    }
                                    else
                                    {
                                        encoreunefois = false;
                                    }
                                }
                                //-- -----------------------------------------------------
                                //-- Creation Vtx (REF:S2(i)  S1(j))    (On Garde S2(i))
                                // OCCT: this block runs even when the one above has just added a vertex; then its
                                // search loop is skipped and its else branch clears encoreunefois, so the loops go
                                // on with the new vertex past nbvtx (reproduced).
                                acreer = false;
                                if (_svtx[i].IsOnDomS2)
                                {
                                    if (_svtx[j].IsOnDomS2)
                                    {
                                        if (!ReferenceEquals(_svtx[j].ArcOnS2(), _svtx[i].ArcOnS2()))
                                        {
                                            acreer = true;
                                        }
                                    }
                                    else
                                    {
                                        acreer = true;
                                    }
                                }
                                if (acreer)
                                {
                                    PatchPoint vtx = _svtx[j];
                                    vtx.SetArc(false, _svtx[i].ArcOnS2(), _svtx[i].ParameterOnArc2(),
                                               _svtx[i].TransitionLineArc2(), _svtx[i].TransitionOnS2());
                                    for (k = 1; !encoreunefois && k <= nbvtx; k++)
                                    {
                                        if (SameVtxRst(_svtx[k], vtx))
                                        {
                                            encoreunefois = true;
                                        }
                                    }
                                    if (!encoreunefois)
                                    {
                                        _svtx[i].SetTolerance(newtoli);
                                        _svtx[j].SetTolerance(newtolj);
                                        vtx.SetTolerance(newtolj);
                                        _svtx.Append(vtx);
                                        encoreunefois = true;
                                    }
                                    else
                                    {
                                        encoreunefois = false;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } while (encoreunefois);

        //-- ----------------------------------------------------------------------

        do
        {
            aPointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!aPointDeleted); i++)
            {
                PatchPoint vtXi = _svtx.Value(i);
                if (vtXi.Tolerance() > tol)
                {
                    tol = vtXi.Tolerance(); //-- 9 oct 97
                }
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
        } while (aPointDeleted);

        nbvtx = VertexCount;
        nbponline = NbPnts();

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

        //----------------------------------------------------
        //-- On detecte les points confondus dans la LineOn2S
        double dmini = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        for (i = 2; (i <= nbponline) && (nbponline > 2); i++)
        {
            PntOn2S pnt1 = _curv.Value(i - 1);
            PntOn2S pnt2 = _curv.Value(i);
            double d = (pnt1.Value()).SquareDistanceTo((pnt2.Value()));
            if (d < dmini)
            {
                _curv.RemovePoint(i);
                nbponline--;
                //----------------------------------------------
                //-- On recadre les Vertex si besoin
                //--
                for (j = 1; j <= nbvtx; j++)
                {
                    indicevertex = _svtx.Value(j).ParameterOnLine();
                    if (indicevertex >= i)
                    {
                        _svtx[j].SetParameter(indicevertex - 1.0);
                    }
                }
                // modified by NIZNHY-PKV Mon Feb 11 09:28:02 2002 f
                i--;
                // modified by NIZNHY-PKV Mon Feb 11 09:28:04 2002 t
            }
        }
        //----------------------------------------------------
        for (i = 1; i <= nbvtx; i++)
        {
            Point3D p = _svtx.Value(i).Value();
            double vTol = _svtx.Value(i).Tolerance();
            indicevertex = _svtx.Value(i).ParameterOnLine();
            indicevertexonline = (int)indicevertex;
            //--------------------------------------------------
            //-- On Compare le vertex avec les points de la ligne
            //-- d indice   indicevertexOnLine-1
            //--            indicevertexOnLine
            //--            indicevertexOnLine+1
            //--------------------------------------------------
            if (indicevertexonline < 1)
            {
                if (CompareVertexAndPoint(p, _curv.Value(1).Value(), vTol))
                {
                    //-------------------------------------------------------
                    //-- On remplace le point cheminement(1) par vertex(i)
                    //-- et   vertex(i) prend pour parametre 1
                    //-------------------------------------------------------

                    PntOn2S pOn2S = _svtx.Value(i).PntOn2S();
                    RecadreMemePeriode(ref pOn2S, _curv.Value(1), U1Period(), V1Period(), U2Period(), V2Period());
                    if (_creationWay == IntPatch_WLType.WLImpImp)
                    {
                        // Adjust first point of curve to corresponding vertex the following way:
                        // set 3D point as the point of the vertex and 2D points as the points of the point on
                        // curve.
                        _curv.SetPoint(1, pOn2S.Value());
                        _curv.Value(1).Parameters(out double mu1, out double mv1, out double mu2, out double mv2);
                        _svtx[i].SetParameter(1);
                        _svtx[i].SetParameters(mu1, mv1, mu2, mv2);
                    }
                    else
                    {
                        _curv.Value(1, pOn2S);
                    }

                    //--curv->Value(1,svtx.Value(i).PntOn2S());
                    _svtx[i].SetParameter(1.0);
                }
                else
                {
                    //-------------------------------------------------------
                    //-- On insere le point de cheminement Vertex(i)
                    //-- On recadre les parametres des autres vertex
                    //-------------------------------------------------------
                    PntOn2S pOn2S = _svtx.Value(i).PntOn2S();
                    RecadreMemePeriode(ref pOn2S, _curv.Value(1), U1Period(), V1Period(), U2Period(), V2Period());
                    _curv.InsertBefore(1, pOn2S);

                    //-- curv->InsertBefore(1,svtx.Value(i).PntOn2S());
                    _svtx[i].SetParameter(1.0);
                    nbponline++;
                    for (j = 1; j <= nbvtx; j++)
                    {
                        if (j != 1) // OCCT: compares j with 1, not with i (reproduced)
                        {
                            double t = _svtx.Value(j).ParameterOnLine();
                            if (t > 1.0)
                            {
                                _svtx[j].SetParameter(t + 1.0);
                            }
                        }
                    }
                }
            } //--- fin : if(indicevertexonline<1)
            else
            {
                //---------------------------------------------------------
                //-- vertex(i)   ==   cheminement (indicevertexonline-1)
                //-- vertex(i)   ==   cheminement (indicevertexonline)
                //-- vertex(i)   ==   cheminement (indicevertexonline+1)
                //---------------------------------------------------------
                bool substitution = false;
                //-- for(k=indicevertexonline+1; !Substitution && k>=indicevertexonline-1;k--) {   avant le 9
                // oct 97
                _curv.Value(indicevertexonline).Parameters(out double mu1, out double mv1, out double mu2, out double mv2);

                for (k = indicevertexonline + 1; k >= indicevertexonline - 1; k--)
                {
                    if (k > 0 && k <= nbponline)
                    {
                        if (CompareVertexAndPoint(p, _curv.Value(k).Value(), vTol))
                        {
                            //-------------------------------------------------------
                            //-- On remplace le point cheminement(k)
                            //-- par vertex(i)  et vertex(i) prend pour parametre k
                            //-------------------------------------------------------
                            PntOn2S pOn2S = _svtx.Value(i).PntOn2S();
                            RecadreMemePeriode(ref pOn2S, _curv.Value(k), U1Period(), V1Period(), U2Period(), V2Period());

                            if (_creationWay == IntPatch_WLType.WLImpImp)
                            {
                                // Adjust a point of curve to corresponding vertex the following way:
                                // set 3D point as the point of the vertex and 2D points as the points
                                // of the point on curve with index <indicevertexonline>
                                _curv.SetPoint(k, pOn2S.Value());
                                _curv.SetUV(k, true, mu1, mv1);
                                _curv.SetUV(k, false, mu2, mv2);
                            }
                            else
                            {
                                _curv.Value(k, pOn2S);
                                pOn2S.Parameters(out mu1, out mv1, out mu2, out mv2);
                            }
                            _svtx[i].SetParameter(k);
                            _svtx[i].SetParameters(mu1, mv1, mu2, mv2);
                            substitution = true;
                        }
                    }
                }

                // Remove duplicating points
                if (substitution)
                {
                    int ind_point;
                    for (ind_point = 2; (ind_point <= nbponline && nbponline > 1); ind_point++)
                    {
                        double d = (_curv.Value(ind_point - 1).Value()).SquareDistanceTo((_curv.Value(ind_point).Value()));
                        if (d < dmini)
                        {
                            _curv.RemovePoint(ind_point);
                            nbponline--;
                            //----------------------------------------------
                            //-- On recadre les Vertex si besoin
                            //--
                            for (j = 1; j <= nbvtx; j++)
                            {
                                indicevertex = _svtx.Value(j).ParameterOnLine();
                                if (indicevertex >= ind_point)
                                {
                                    _svtx[j].SetParameter(indicevertex - 1.0);
                                }
                            }
                            // modified by NIZNHY-PKV Mon Feb 11 09:28:02 2002 f
                            ind_point--;
                            // modified by NIZNHY-PKV Mon Feb 11 09:28:04 2002 t
                        }
                    }
                }

                //--static int deb6nov98=1;    Ne resout rien (a part partiellement BUC60409)
                //--if(deb6nov98) {
                //--Substitution=true;
                //-- }

                if (!substitution)
                {
                    //-------------------------------------------------------
                    //-- On insere le point de cheminement Vertex(i)
                    //-- On recadre les parametres des autres vertex
                    //-------------------------------------------------------
                    PntOn2S pOn2S = _svtx.Value(i).PntOn2S();
                    if (indicevertexonline >= nbponline)
                    {
                        RecadreMemePeriode(ref pOn2S, _curv.Value(nbponline), U1Period(), V1Period(), U2Period(),
                                           V2Period());
                        _curv.Add(pOn2S);
                    }
                    else
                    {
                        RecadreMemePeriode(ref pOn2S, _curv.Value(indicevertexonline + 1), U1Period(), V1Period(),
                                           U2Period(), V2Period());
                        _curv.InsertBefore(indicevertexonline + 1, pOn2S);
                    }
                    //-- curv->InsertBefore(indicevertexonline+1,svtx.Value(i).PntOn2S());
                    _svtx[i].SetParameter(indicevertexonline + 1);
                    nbponline++;
                    for (j = 1; j <= nbvtx; j++)
                    {
                        if (j != i)
                        {
                            double t = _svtx.Value(j).ParameterOnLine();
                            if (t > (double)indicevertexonline)
                            {
                                _svtx[j].SetParameter(t + 1.0);
                            }
                        }
                    }
                } //-- Substitution
            } //-- indicevertexonline>=1

        } //-- boucle i sur vertex

        do
        {
            aPointDeleted = false;
            for (i = 1; i <= nbvtx && (!aPointDeleted); i++)
            {
                PatchPoint VTX = _svtx.Value(i);
                for (j = 1; j <= nbvtx && (!aPointDeleted); j++)
                {
                    if (i != j)
                    {
                        PatchPoint vtxm1 = _svtx.Value(j);

                        bool kill = false;
                        bool killm1 = false;
                        if (vtxm1.ParameterOnLine() == VTX.ParameterOnLine())
                        {
                            if (vtxm1.IsOnDomS1 && VTX.IsOnDomS1)
                            { //-- OnS1    OnS1
                                if (ReferenceEquals(vtxm1.ArcOnS1(), VTX.ArcOnS1()))
                                { //-- OnS1 == OnS1
                                    if (vtxm1.IsOnDomS2)
                                    { //-- OnS1 == OnS1  OnS2
                                        if (!VTX.IsOnDomS2)
                                        { //-- OnS1 == OnS1  OnS2 PasOnS2
                                            kill = true;
                                        }
                                        else
                                        {
                                            if (ReferenceEquals(vtxm1.ArcOnS2(), VTX.ArcOnS2()))
                                            { //-- OnS1 == OnS1  OnS2 == OnS2
                                                kill = true;
                                            }
                                        }
                                    }
                                    else
                                    { //-- OnS1 == OnS1  PasOnS2
                                        if (VTX.IsOnDomS2)
                                        { //-- OnS1 == OnS1  PasOnS2  OnS2
                                            killm1 = true;
                                        }
                                    }
                                }
                            }

                            if (!(kill || killm1))
                            {
                                if (vtxm1.IsOnDomS2 && VTX.IsOnDomS2)
                                { //-- OnS2    OnS2
                                    if (ReferenceEquals(vtxm1.ArcOnS2(), VTX.ArcOnS2()))
                                    { //-- OnS2 == OnS2
                                        if (vtxm1.IsOnDomS1)
                                        { //-- OnS2 == OnS2  OnS1
                                            if (!VTX.IsOnDomS1)
                                            { //-- OnS2 == OnS2  OnS1 PasOnS1
                                                kill = true;
                                            }
                                            else
                                            {
                                                if (ReferenceEquals(vtxm1.ArcOnS1(), VTX.ArcOnS1()))
                                                { //-- OnS2 == OnS2  OnS1 == OnS1
                                                    kill = true;
                                                }
                                            }
                                        }
                                        else
                                        { //-- OnS2 == OnS2  PasOnS1
                                            if (VTX.IsOnDomS1)
                                            { //-- OnS2 == OnS2  PasOnS1  OnS1
                                                killm1 = true;
                                            }
                                        }
                                    }
                                }
                            }
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
        //-- ----------------------------------------------------------------------------
        //-- On ajoute les vertex de debut et de fin de ligne s il ne sont pas presents.
        //--
        //-- Existe t il un vertex de debut de ligne, de fin .
        //--
        //-- Si Besoin : il faudra dedoubler les points de debut et de fin sur les periodiques ??????

        bool bFirst = false;
        bool bLast = false;
        nbponline = NbPnts();
        for (i = 1; i <= nbvtx; i++)
        {
            double pol = _svtx.Value(i).ParameterOnLine();
            if (pol == 1.0)
            {
                bFirst = _fipt = true;
                _indf = i;
            }
            if (pol == nbponline)
            {
                bLast = _lapt = true;
                _indl = i;
            }
        }
        if (!bFirst)
        {
            bool vtxfound = false;
            PatchPoint vtx = new();
            _curv.Value(1).Parameters(out double pu1, out double pv1, out double pu2, out double pv2);

            for (i = 1; (!vtxfound) && (i <= nbvtx); i++)
            {
                PatchPoint v = _svtx.Value(i);
                // jgv: to avoid loops
                // double vTol = V.Tolerance();
                if (CompareVertexAndPoint(v.Value(), _curv.Value(1).Value(), global::Kerf.Engine.Foundation.Precision.Confusion() /*vTol*/))
                {
                    vtx = v;
                    vtx.SetParameters(pu1, pv1, pu2, pv2);
                    vtxfound = true;
                }
            }
            if (!vtxfound)
            {
                vtx.SetValue(_curv.Value(1).Value(), tol, false);
                vtx.SetParameters(pu1, pv1, pu2, pv2);
            }
            vtx.SetParameter(1);
            _svtx.Prepend(vtx);
            nbvtx++;
            _fipt = true;
            _indf = 1;
        }
        if (!bLast)
        {
            bool vtxfound = false;
            PatchPoint vtx = new();
            _curv.Value(nbponline).Parameters(out double pu1, out double pv1, out double pu2, out double pv2);

            for (i = 1; (!vtxfound) && (i <= nbvtx); i++)
            {
                PatchPoint v = _svtx.Value(i);
                // jgv: to avoid loops
                // double vTol = V.Tolerance();
                if (CompareVertexAndPoint(v.Value(), _curv.Value(nbponline).Value(), global::Kerf.Engine.Foundation.Precision.Confusion() /*vTol*/))
                {
                    vtx = v;
                    vtx.SetParameters(pu1, pv1, pu2, pv2);
                    vtxfound = true;
                }
            }
            if (!vtxfound)
            {
                vtx.SetValue(_curv.Value(nbponline).Value(), tol, false);
                vtx.SetParameters(pu1, pv1, pu2, pv2);
            }
            vtx.SetParameter(nbponline);
            _svtx.Append(vtx);
            nbvtx++;
            _lapt = true;
            _indl = nbvtx;
        }

        //--------------------------------------------------------------
        //-- ** Detection de points trouves sur une meme restriction
        //--    avec la meme transition et avec des params on line
        //--    voisins.
        //-- ** Dans ce cas (-> donnerait un baillemenmt) on supprime
        //--    le point 'intermediaire'.
        //-- ** (exemple Vtx(1)  .....  Vtx(204) Vtx(205))
        //--    on supprime le Vtx(204)
        //-- ** (exemple Vtx(1) Vtx(2)  .....  Vtx(205))
        //--    on supprime le Vtx(2)
        //-- ** (exemple Vtx(1)  ...  Vtx(100) Vtx(101) ... Vtx(205))
        //--    on supprime le Vtx(100)  (Vtx(100)et101 sur m restr)
        //--------------------------------------------------------------
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
                                if ((vtXi.ParameterOnLine() == vtXj.ParameterOnLine() + 1)
                                    || (vtXi.ParameterOnLine() == vtXj.ParameterOnLine() - 1))
                                {
                                    if (ReferenceEquals(vtXi.ArcOnS1(), vtXj.ArcOnS1()))
                                    {
                                        WalkingTransition t1 = vtXi.TransitionLineArc1();
                                        WalkingTransition t2 = vtXj.TransitionLineArc1();
                                        if (t1.TransitionType() == t2.TransitionType())
                                        {
                                            if ((vtXj.ParameterOnLine() != 1) && (vtXj.ParameterOnLine() != NbPnts()))
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
            }
        }
        //-- meme traitement sur les restrictions du second shape
        while (aPointDeleted);

        nbvtx = VertexCount;
        do
        {
            aPointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!aPointDeleted); i++)
            {
                PatchPoint vtXi = _svtx.Value(i);
                if ((!vtXi.IsOnDomS1) && (vtXi.IsOnDomS2))
                {
                    for (j = 1; (j <= nbvtx) && (!aPointDeleted); j++)
                    {
                        if (i != j)
                        {
                            PatchPoint vtXj = _svtx.Value(j);
                            if ((!vtXj.IsOnDomS1) && (vtXj.IsOnDomS2))
                            {
                                if ((vtXi.ParameterOnLine() == vtXj.ParameterOnLine() + 1)
                                    || (vtXi.ParameterOnLine() == vtXj.ParameterOnLine() - 1))
                                {
                                    if (ReferenceEquals(vtXi.ArcOnS2(), vtXj.ArcOnS2()))
                                    {
                                        WalkingTransition t1 = vtXi.TransitionLineArc2();
                                        WalkingTransition t2 = vtXj.TransitionLineArc2();
                                        if (t1.TransitionType() == t2.TransitionType())
                                        {
                                            if ((vtXj.ParameterOnLine() != 1) && (vtXj.ParameterOnLine() != NbPnts()))
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
            }
        } while (aPointDeleted);
        //--------------------------------------------------------------

        //--------------------------------------------------------------
        //-- dans le cas de lignes periodiques du type :
        //--    Un point sur restriction R1 de param p1  -> P3d1   Vtx1
        //--    Un point sur restriction R2 de param p2  -> P3d1   Vtx2
        //--
        //--    Un point sur restriction R1 de param p1  -> P3d1   Vtx3
        //--    pas de point sur R2
        //--
        //-- On doit dans ce cas creer un nouveau Vtx4 = Vtx3 sur la
        //-- restriction R2
        //--
        //-- Ce cas se produit qd on a corrige un baillement avec le filtre
        //-- precedent
        //--

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

        // DEBUGV: a dump of the vertices after the computation, compiled out in OCCT: left out.
    }

    /// <summary>
    /// Writes the line as OCCT's <c>Dump</c> prints it: mode 0 the points and vertices, 1 the 3D points, 2 the points
    /// on the first surface, others the points on the second surface.
    /// </summary>
    public void Dump(int mode, TextWriter theOut) => DumpPointLine(this, " W", mode, theOut);

    // The body of WLine's and RLine's Dump, which differ only in the title.
    internal static void DumpPointLine(PointLine line, string title, int mode, TextWriter theOut)
    {
        theOut.Write(" ----------- D u m p    I n t P a t c h  _ " + title + " L i n e  -(begin)------" + '\n');
        int nbPoints = line.NbPnts();
        int nbVertex = line.VertexCount;

        switch (mode)
        {
            case 0:
                theOut.Write("Num    [X  Y  Z]     [U1  V1]   [U2  V2]\n");
                for (int i = 1; i <= nbPoints; i++)
                {
                    PntOn2S p = line.Point(i);
                    p.Parameters(out double u1, out double v1, out double u2, out double v2);
                    theOut.Write($"{i.ToString(CultureInfo.InvariantCulture),4}  [{F(p.Value().X)} {F(p.Value().Y)} "
                                 + $"{F(p.Value().Z)}]  [{F(u1)} {F(v1)}]  [{F(u2)} {F(v2)}]\n");
                }

                for (int i = 1; i <= nbVertex; i++)
                {
                    line.Vertex(i).Dump(theOut);
                    double polr = line.Vertex(i).ParameterOnLine();
                    int pol = (int)polr;

                    // OCCT: indexes the vertices with the point's rank on the line (Vertex(pol), not Point(pol)),
                    // and so only when that rank is at most the number of vertices (reproduced).
                    if (pol >= 1 && pol <= nbVertex)
                    {
                        Point3D v = line.Vertex(pol).Value();
                        theOut.Write("----> IntSurf_PntOn2S : " + R(polr) + ", Pnt (" + R(v.X) + "," + R(v.Y)
                                     + "," + R(v.Z) + ")" + '\n');
                    }
                }
                break;
            case 1:
                for (int i = 1; i <= nbPoints; i++)
                {
                    Point3D p = line.Point(i).Value();
                    theOut.Write($"point p{i.ToString(CultureInfo.InvariantCulture)} {F(p.X)} {F(p.Y)} {F(p.Z)}\n");
                }
                break;
            case 2:
                for (int i = 1; i <= nbPoints; i++)
                {
                    line.Point(i).Parameters(out double u1, out double v1, out _, out _);
                    theOut.Write($"point p{i.ToString(CultureInfo.InvariantCulture)} {F(u1)} {F(v1)}\n");
                }
                break;
            default:
                for (int i = 1; i <= nbPoints; i++)
                {
                    line.Point(i).Parameters(out _, out _, out double u2, out double v2);
                    theOut.Write($"point p{i.ToString(CultureInfo.InvariantCulture)} {F(u2)} {F(v2)}\n");
                }
                break;
        }
        theOut.Write("\n--------------------------------------------------- (end) -------" + '\n');
    }

    // printf's "%+10.20f": a sign always, twenty decimals.
    private static string F(double value)
    {
        string str = value.ToString("F20", CultureInfo.InvariantCulture);
        return str.StartsWith('-') ? str : "+" + str;
    }

    // std::cout << double: six significant digits.
    private static string R(double value) => StreamUtilities.FormatReal(value, 6);
}
