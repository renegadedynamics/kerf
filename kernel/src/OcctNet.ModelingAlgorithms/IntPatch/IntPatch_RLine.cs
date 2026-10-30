// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_RLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_RLine.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_RLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntPatch_IType;

namespace Occt;

/// <summary>
/// An intersection line that is a restriction (an arc) of one of the surfaces, with its vertices and, optionally, a
/// polygon of points (<see cref="Curve"/>).
/// </summary>
/// <remarks>
/// A working object (decision 609). OCCT declares <c>SetParamOnS1</c> and <c>SetParamOnS2</c> but never defines
/// them: left out. <see cref="Dump(int, TextWriter)"/> is OCCT's <c>Dump</c> (to stdout).
/// </remarks>
public class IntPatch_RLine : IntPatch_PointLine
{
    private Adaptor2d_Curve2d? theArcOnS1;
    private Adaptor2d_Curve2d? theArcOnS2;
    private bool onS1;
    private bool onS2;
    // ParamInf1, ParamSup1, ParamInf2, ParamSup2: initialised to 0 by OCCT and never read: left out (decision 143).
    private IntSurf_LineOn2S? curv;
    private bool fipt;
    private bool lapt;
    private int indf;
    private int indl;
    private readonly NCollection_Sequence<IntPatch_Point> svtx = new();

    /// <summary>A restriction line whose transitions are in or out.</summary>
    public IntPatch_RLine(bool Tang, IntSurf_TypeTrans Trans1, IntSurf_TypeTrans Trans2)
        : base(Tang, Trans1, Trans2)
    {
        fipt = false;
        lapt = false;
        indf = 0;
        indl = 0;
        typ = IntPatch_Restriction;
        onS2 = false;
        onS1 = false;
    }

    /// <summary>A restriction line whose transitions are touch.</summary>
    public IntPatch_RLine(bool Tang, IntSurf_Situation Situ1, IntSurf_Situation Situ2)
        : base(Tang, Situ1, Situ2)
    {
        fipt = false;
        lapt = false;
        indf = 0;
        indl = 0;
        typ = IntPatch_Restriction;
        onS2 = false;
        onS1 = false;
    }

    /// <summary>A restriction line whose transitions are undecided.</summary>
    public IntPatch_RLine(bool Tang)
        : base(Tang)
    {
        fipt = false;
        lapt = false;
        indf = 0;
        indl = 0;
        typ = IntPatch_Restriction;
        onS2 = false;
        onS1 = false;
    }

    public override void AddVertex(in IntPatch_Point Pnt, bool theIsPrepend = false)
    {
        if (theIsPrepend)
        {
            svtx.Prepend(Pnt);
        }
        else
        {
            svtx.Append(Pnt);
        }
    }

    /// <summary>Replaces the vertex of range <paramref name="Index"/>.</summary>
    public void Replace(int Index, in IntPatch_Point Pnt) => svtx[Index] = Pnt;

    public void SetFirstPoint(int IndFirst)
    {
        fipt = true;
        indf = IndFirst;
    }

    public void SetLastPoint(int IndLast)
    {
        lapt = true;
        indl = IndLast;
    }

    /// <summary>Sets the polygon of points of the line (kept, not copied, as OCCT's handle).</summary>
    public void Add(IntSurf_LineOn2S L) => curv = L;

    /// <summary>True if the line is an arc of restriction of the first surface.</summary>
    public bool IsArcOnS1() => onS1;

    /// <summary>True if the line is an arc of restriction of the second surface.</summary>
    public bool IsArcOnS2() => onS2;

    public void SetArcOnS1(Adaptor2d_Curve2d A)
    {
        theArcOnS1 = A;
        onS1 = true;
    }

    public void SetArcOnS2(Adaptor2d_Curve2d A)
    {
        theArcOnS2 = A;
        onS2 = true;
    }

    public Adaptor2d_Curve2d? ArcOnS1() => theArcOnS1;

    public Adaptor2d_Curve2d? ArcOnS2() => theArcOnS2;

    /// <summary>
    /// The smallest and largest parameters on the line of the vertices if the line is on the first surface's arc;
    /// otherwise zeros.
    /// </summary>
    public void ParamOnS1(out double a, out double b)
    {
        if (onS1)
        {
            a = RealLast();
            b = -a;
            for (int i = svtx.Length(); i >= 1; i--)
            {
                double p = svtx.Value(i).ParameterOnLine();
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
        if (onS2)
        {
            a = RealLast();
            b = -a;
            for (int i = svtx.Length(); i >= 1; i--)
            {
                double p = svtx.Value(i).ParameterOnLine();
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

    public bool HasFirstPoint() => fipt;

    public bool HasLastPoint() => lapt;

    /// <summary>The vertex that is the first point; throws Standard_DomainError if there is none.</summary>
    public IntPatch_Point FirstPoint()
    {
        if (!fipt)
        {
            throw new Standard_DomainError();
        }
        return svtx.Value(indf);
    }

    /// <summary>The vertex that is the last point; throws Standard_DomainError if there is none.</summary>
    public IntPatch_Point LastPoint()
    {
        if (!lapt)
        {
            throw new Standard_DomainError();
        }
        return svtx.Value(indl);
    }

    public override int NbVertex() => svtx.Length();

    public override IntPatch_Point Vertex(int Index) => svtx.Value(Index);

    public override ref IntPatch_Point ChangeVertex(int Index) => ref svtx[Index];

    public override void RemoveVertex(int theIndex)
    {
        if ((theIndex < 1) || (theIndex > NbVertex()))
        {
            throw new Standard_OutOfRange("Cannot delete not existing vertex");
        }
        svtx.Remove(theIndex);
    }

    /// <summary>True if the line has a polygon of points.</summary>
    public bool HasPolygon() => (curv is not null);

    /// <summary>The number of points of the polygon; throws Standard_DomainError if there is none.</summary>
    public override int NbPnts()
    {
        if (curv is null)
        {
            throw new Standard_DomainError();
        }
        return curv.NbPoints();
    }

    /// <summary>The point of range <paramref name="Index"/> of the polygon; throws Standard_DomainError if there is none.</summary>
    public override IntSurf_PntOn2S Point(int Index)
    {
        if (curv is null)
        {
            throw new Standard_DomainError();
        }
        return curv.Value(Index);
    }

    /// <summary>Sets the point of range <paramref name="Index"/> of the polygon to the vertex's point and parameters.</summary>
    public void SetPoint(int Index, in IntPatch_Point thepoint) => curv!.Value(Index, thepoint.PntOn2S());

    public override IntSurf_LineOn2S? Curve() => (curv);

    public override bool IsOutSurf1Box(in gp_Pnt2d theP) => curv!.IsOutSurf1Box(theP);

    public override bool IsOutSurf2Box(in gp_Pnt2d theP) => curv!.IsOutSurf2Box(theP);

    public override bool IsOutBox(in gp_Pnt theP) => curv!.IsOutBox(theP);

    public override void ClearVertexes() => svtx.Clear();

    /// <summary>Replaces the polygon of points.</summary>
    public void SetCurve(IntSurf_LineOn2S theNewCurve) => curv = theNewCurve;

    /// <summary>
    /// Removes the duplicate vertices (same arc and parameter, or the vertex on fewer arcs at the same parameter), sorts
    /// them by parameter on the line, and makes the first and the last the line's first and last points. The
    /// tolerance is not used (as in OCCT).
    /// </summary>
    public void ComputeVertexParameters(double Tol)
    {
        int i, j, nbvtx; // k;

        bool APointDeleted = false;
        //----------------------------------------------------------
        //--     F i l t r e   s u r   r e s t r i c t i o n s   --
        //----------------------------------------------------------
        //-- deux vertex sur la meme restriction et seulement
        //-- sur celle ci ne doivent pas avoir le meme parametre
        //--
        nbvtx = NbVertex();
        do
        {
            APointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!APointDeleted); i++)
            {
                IntPatch_Point VTXi = svtx.Value(i);
                if ((VTXi.IsOnDomS1()) && (!VTXi.IsOnDomS2()))
                {
                    for (j = 1; (j <= nbvtx) && (!APointDeleted); j++)
                    {
                        if (i != j)
                        {
                            IntPatch_Point VTXj = svtx.Value(j);
                            if ((VTXj.IsOnDomS1()) && (!VTXj.IsOnDomS2()))
                            {
                                if (VTXi.ParameterOnLine() == VTXj.ParameterOnLine())
                                {
                                    if (ReferenceEquals(VTXi.ArcOnS1(), VTXj.ArcOnS1()))
                                    {
                                        if (VTXi.ParameterOnArc1() == VTXj.ParameterOnArc1())
                                        {
                                            svtx.Remove(j);
                                            nbvtx--;
                                            if (lapt)
                                            {
                                                if (indl >= j)
                                                {
                                                    indl--;
                                                }
                                            }
                                            if (fipt)
                                            {
                                                if (indf >= j)
                                                {
                                                    indf--;
                                                }
                                            }
                                            APointDeleted = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } while (APointDeleted);

        do
        {
            APointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!APointDeleted); i++)
            {
                IntPatch_Point VTXi = svtx.Value(i);
                if ((VTXi.IsOnDomS2()) && (!VTXi.IsOnDomS1()))
                {
                    for (j = 1; (j <= nbvtx) && (!APointDeleted); j++)
                    {
                        if (i != j)
                        {
                            IntPatch_Point VTXj = svtx.Value(j);
                            if ((VTXj.IsOnDomS2()) && (!VTXj.IsOnDomS1()))
                            {
                                if (VTXi.ParameterOnLine() == VTXj.ParameterOnLine())
                                {
                                    if (ReferenceEquals(VTXi.ArcOnS2(), VTXj.ArcOnS2()))
                                    {
                                        if (VTXi.ParameterOnArc2() == VTXj.ParameterOnArc2())
                                        {
                                            svtx.Remove(j);
                                            nbvtx--;
                                            if (lapt)
                                            {
                                                if (indl >= j)
                                                {
                                                    indl--;
                                                }
                                            }
                                            if (fipt)
                                            {
                                                if (indf >= j)
                                                {
                                                    indf--;
                                                }
                                            }
                                            APointDeleted = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } while (APointDeleted);

        nbvtx = NbVertex();

        //----------------------------------------------------
        //-- On trie les Vertex
        bool SortIsOK;
        do
        {
            SortIsOK = true;
            for (i = 2; i <= nbvtx; i++)
            {
                if (svtx.Value(i - 1).ParameterOnLine() > svtx.Value(i).ParameterOnLine())
                {
                    SortIsOK = false;
                    svtx.Exchange(i - 1, i);
                }
            }
        } while (!SortIsOK);

        do
        {
            APointDeleted = false;
            bool restrdiff;
            for (i = 1; i <= nbvtx && (!APointDeleted); i++)
            {
                IntPatch_Point VTX = svtx.Value(i);
                for (j = 1; j <= nbvtx && (!APointDeleted); j++)
                {
                    if (i != j)
                    {
                        IntPatch_Point VTXM1 = svtx.Value(j);

                        bool kill = false;
                        bool killm1 = false;
                        if (VTXM1.ParameterOnLine() == VTX.ParameterOnLine())
                        {
                            restrdiff = false;
                            if (VTXM1.IsOnDomS1() && VTX.IsOnDomS1())
                            { //-- OnS1    OnS1
                                if (ReferenceEquals(VTXM1.ArcOnS1(), VTX.ArcOnS1()))
                                { //-- OnS1 == OnS1
                                    if (VTX.ParameterOnArc1() == VTXM1.ParameterOnArc1())
                                    {
                                        if (VTXM1.IsOnDomS2())
                                        { //-- OnS1 == OnS1  OnS2
                                            if (!VTX.IsOnDomS2())
                                            { //-- OnS1 == OnS1  OnS2 PasOnS2
                                                kill = true;
                                            }
                                            else
                                            {
                                                if (ReferenceEquals(VTXM1.ArcOnS2(), VTX.ArcOnS2()))
                                                { //-- OnS1 == OnS1  OnS2 == OnS2
                                                    if (VTX.ParameterOnArc2() == VTXM1.ParameterOnArc2())
                                                    {
                                                        kill = true;
                                                    }
                                                }
                                            }
                                        }
                                        else
                                        { //-- OnS1 == OnS1  PasOnS2
                                            if (VTX.IsOnDomS2())
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
                                if (VTXM1.IsOnDomS2() && VTX.IsOnDomS2())
                                { //-- OnS2    OnS2
                                    if (ReferenceEquals(VTXM1.ArcOnS2(), VTX.ArcOnS2()))
                                    { //-- OnS2 == OnS2
                                        if (VTX.ParameterOnArc2() == VTXM1.ParameterOnArc2())
                                        {
                                            if (VTXM1.IsOnDomS1())
                                            { //-- OnS2 == OnS2  OnS1
                                                if (!VTX.IsOnDomS1())
                                                { //-- OnS2 == OnS2  OnS1 PasOnS1
                                                    kill = true;
                                                }
                                                else
                                                {
                                                    if (ReferenceEquals(VTXM1.ArcOnS1(), VTX.ArcOnS1()))
                                                    { //-- OnS2 == OnS2  OnS1 == OnS1
                                                        if (VTX.ParameterOnArc1() == VTXM1.ParameterOnArc1())
                                                        {
                                                            kill = true;
                                                        }
                                                    }
                                                }
                                            }
                                            else
                                            { //-- OnS2 == OnS2  PasOnS1
                                                if (VTX.IsOnDomS1())
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
                                    APointDeleted = true;
                                    svtx.Remove(i);
                                    nbvtx--;
                                }
                                else if (killm1)
                                {
                                    APointDeleted = true;
                                    svtx.Remove(j);
                                    nbvtx--;
                                }
                            }
                        }
                    }
                }
            }
        } while (APointDeleted);

        do
        {
            SortIsOK = true;
            for (i = 2; i <= nbvtx && SortIsOK; i++)
            {
                IntPatch_Point Pim1 = svtx.Value(i - 1);
                IntPatch_Point Pii = svtx.Value(i);
                if (Pim1.ParameterOnLine() == Pii.ParameterOnLine())
                {
                    if ((!Pii.IsOnDomS1()) && (!Pii.IsOnDomS2()))
                    {
                        SortIsOK = false;
                        svtx.Remove(i);
                        nbvtx--;
                    }
                    else
                    {
                        if ((!Pim1.IsOnDomS1()) && (!Pim1.IsOnDomS2()))
                        {
                            SortIsOK = false;
                            svtx.Remove(i - 1);
                            nbvtx--;
                        }
                    }
                }
            }
        } while (!SortIsOK);

        //----------------------------------------------------
        //-- On trie les Vertex ( Cas Bizarre )
        nbvtx = NbVertex();
        do
        {
            SortIsOK = true;
            for (i = 2; i <= nbvtx; i++)
            {
                if (svtx.Value(i - 1).ParameterOnLine() > svtx.Value(i).ParameterOnLine())
                {
                    SortIsOK = false;
                    svtx.Exchange(i - 1, i);
                }
            }
        } while (!SortIsOK);

        SetFirstPoint(1);
        SetLastPoint(nbvtx);
    }

    /// <summary>
    /// Writes the line as OCCT's <c>Dump</c> prints it (see <see cref="IntPatch_WLine.Dump(int, TextWriter)"/>);
    /// throws Standard_DomainError if the line has no polygon.
    /// </summary>
    public void Dump(int theMode, TextWriter theOut) => IntPatch_WLine.DumpPointLine(this, " R", theMode, theOut);
}
