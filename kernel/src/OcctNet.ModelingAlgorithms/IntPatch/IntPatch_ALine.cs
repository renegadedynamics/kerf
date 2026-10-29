// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALine.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntPatch_IType;

namespace Occt;

/// <summary>
/// An intersection line described by a parametrised curve (<see cref="IntAna_Curve"/>, the intersection of a
/// cylinder or a cone with a quadric), with its vertices.
/// </summary>
/// <remarks>
/// A working object (decision 5103). The curve is a struct: the line keeps a copy, as OCCT does, and
/// <see cref="Curve"/> returns a copy. <see cref="Vertex"/>, <see cref="FirstPoint"/> and <see cref="LastPoint"/>
/// return copies; <see cref="ChangeVertex"/> returns a reference to the stored vertex.
/// </remarks>
public class IntPatch_ALine : IntPatch_Line
{
    private const double PCONFUSION = 0.00001;

    private readonly IntAna_Curve curv;
    private bool fipt;
    private bool lapt;
    private int indf;
    private int indl;
    private readonly NCollection_Sequence<IntPatch_Point> svtx = new();

    /// <summary>Creates an analytic intersection line when the transitions are in or out.</summary>
    public IntPatch_ALine(in IntAna_Curve C, bool Tang, IntSurf_TypeTrans Trans1, IntSurf_TypeTrans Trans2)
        : base(Tang, Trans1, Trans2)
    {
        typ = IntPatch_Analytic;
        curv = C;
    }

    /// <summary>Creates an analytic intersection line when the transitions are touch.</summary>
    public IntPatch_ALine(in IntAna_Curve C, bool Tang, IntSurf_Situation Situ1, IntSurf_Situation Situ2)
        : base(Tang, Situ1, Situ2)
    {
        typ = IntPatch_Analytic;
        curv = C;
    }

    /// <summary>Creates an analytic intersection line when the transitions are undecided.</summary>
    public IntPatch_ALine(in IntAna_Curve C, bool Tang)
        : base(Tang)
    {
        typ = IntPatch_Analytic;
        curv = C;
    }

    /// <summary>Adds a vertex at the end of the list.</summary>
    public void AddVertex(in IntPatch_Point VTXj) => svtx.Append(VTXj);

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

    /// <summary>
    /// The first parameter on the line; <paramref name="IsIncluded"/> if <see cref="Value"/> and <see cref="D1"/> may
    /// be called with it (otherwise only with greater parameters).
    /// </summary>
    public double FirstParameter(out bool IsIncluded)
    {
        curv.Domain(out double first, out _);
        IsIncluded = !curv.IsFirstOpen();
        return first;
    }

    /// <summary>
    /// The last parameter on the line; <paramref name="IsIncluded"/> if <see cref="Value"/> and <see cref="D1"/> may
    /// be called with it (otherwise only with smaller parameters).
    /// </summary>
    public double LastParameter(out bool IsIncluded)
    {
        curv.Domain(out _, out double last);
        IsIncluded = !curv.IsLastOpen();
        return last;
    }

    /// <summary>The point of parameter <paramref name="U"/> (Standard_DomainError out of the domain).</summary>
    public gp_Pnt Value(double U) => curv.Value(U); // Value leve l exception DomainError

    /// <summary>
    /// The point at <paramref name="U"/>, and true with the derivative <paramref name="Du"/> where it is defined.
    /// </summary>
    public bool D1(double U, out gp_Pnt P, out gp_Vec Du) => curv.D1u(U, out P, out Du); // D1u leve l exception DomainError

    /// <summary>The parameters of the point <paramref name="theP"/> on the curve, sorted ascending (none if it is not on it).</summary>
    public void FindParameter(in gp_Pnt theP, NCollection_List<double> theParams) => curv.FindParameter(theP, theParams);

    /// <summary>True if the line has a known first point (<see cref="FirstPoint"/>).</summary>
    public bool HasFirstPoint() => fipt;

    /// <summary>True if the line has a known last point (<see cref="LastPoint"/>).</summary>
    public bool HasLastPoint() => lapt;

    /// <summary>The first point; throws Standard_DomainError if there is none.</summary>
    public IntPatch_Point FirstPoint()
    {
        if (!fipt)
        {
            throw new Standard_DomainError();
        }
        return svtx.Value(indf);
    }

    /// <summary>The last point; throws Standard_DomainError if there is none.</summary>
    public IntPatch_Point LastPoint()
    {
        if (!lapt)
        {
            throw new Standard_DomainError();
        }
        return svtx.Value(indl);
    }

    public int NbVertex() => svtx.Length();

    /// <summary>The vertex of range <paramref name="Index"/>.</summary>
    public IntPatch_Point Vertex(int Index) => svtx.Value(Index);

    /// <summary>The vertex of range <paramref name="theIndex"/>, to change in place.</summary>
    public ref IntPatch_Point ChangeVertex(int theIndex) => ref svtx.ChangeValue(theIndex);

    /// <summary>The curve (a copy).</summary>
    public IntAna_Curve Curve() => (curv);

    /// <summary>
    /// Sets the parameters of all the vertices on the line: adds the other representations of a vertex on a line
    /// longer than a period, sorts them, removes the duplicates, handles closed lines, drops the end vertices on no
    /// restriction and merges the vertices of equal parameters.
    /// </summary>
    public void ComputeVertexParameters(double Tol)
    {
        bool SortIsOK, APointDeleted;
        bool SortAgain = true;
        int nbvtx, i, j;
        double ParamMinOnLine, ParamMaxOnLine;
        bool OpenFirst, OpenLast;

        ParamMinOnLine = FirstParameter(out OpenFirst);
        ParamMaxOnLine = LastParameter(out OpenLast);

        //----------------------------------------------------------
        //--     F i l t r e   s u r   r e s t r i c t i o n s   --
        //----------------------------------------------------------
        //-- deux vertex sur la meme restriction et seulement
        //-- sur celle ci ne doivent pas avoir le meme parametre
        //--
        //--

        //-- Le tri est necessaire si suppression du first ou du last point
        nbvtx = NbVertex();

        //-- On verifie qu un vertex a bien toute ses representations :
        //-- Cas tres rare : point de tangence sur un debut de ligne
        //-- et la ligne fait 2 * 2 PI de parametrage.

        for (i = 1; i <= nbvtx; i++)
        {
            // C++ holds a reference; the vertex is only read while the sequence grows, so a copy is the same.
            IntPatch_Point VTX = svtx.Value(i);
            double p = VTX.ParameterOnLine();
            double pmpimpi = p - M_PI - M_PI;
            if (pmpimpi >= ParamMinOnLine)
            {
                gp_Pnt P1 = Value(pmpimpi);
                double d1 = P1.Distance(VTX.Value());
                if (d1 < Tol)
                {
                    IntPatch_Point OVTX = VTX;
                    OVTX.SetParameter(pmpimpi);
                    svtx.Append(OVTX);
                }
            }
            pmpimpi = p + M_PI + M_PI;
            if (pmpimpi <= ParamMaxOnLine)
            {
                gp_Pnt P1 = Value(pmpimpi);
                double d1 = P1.Distance(VTX.Value());
                if (d1 < Tol)
                {
                    IntPatch_Point OVTX = VTX;
                    OVTX.SetParameter(pmpimpi);
                    svtx.Append(OVTX);
                }
            }
        }

        nbvtx = NbVertex();

        if (nbvtx <= 0)
        {
            return;
        }
        do
        {
            SortIsOK = true;
            for (i = 2; i <= nbvtx; i++)
            {
                if (svtx.Value(i - 1).ParameterOnLine() > svtx.Value(i).ParameterOnLine())
                {
                    SortIsOK = false;
                    svtx.Exchange(i - 1, i);
                    ExchangeIndices(i);
                }
            }
        } while (!SortIsOK);

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
                                if (Math.Abs(VTXi.ParameterOnArc1() - VTXj.ParameterOnArc1()) <= PCONFUSION)
                                {
                                    if (Math.Abs(VTXi.ParameterOnLine() - VTXj.ParameterOnLine()) <= PCONFUSION)
                                    {
                                        if (ReferenceEquals(VTXi.ArcOnS1(), VTXj.ArcOnS1()))
                                        {
                                            RemoveDuplicate(j, ref nbvtx);
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
                                if (Math.Abs(VTXi.ParameterOnArc2() - VTXj.ParameterOnArc2()) <= PCONFUSION)
                                {
                                    if (Math.Abs(VTXi.ParameterOnLine() - VTXj.ParameterOnLine()) <= PCONFUSION)
                                    {
                                        if (ReferenceEquals(VTXi.ArcOnS2(), VTXj.ArcOnS2()))
                                        {
                                            RemoveDuplicate(j, ref nbvtx);
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

        //----------------------------------------------------------
        //-- Tri des vertex et suppression des Vtx superflus
        //--
        do
        {
            nbvtx = NbVertex();
            if (SortAgain)
            {
                do
                {
                    SortIsOK = true;
                    for (i = 2; i <= nbvtx; i++)
                    {
                        if (svtx.Value(i - 1).ParameterOnLine() > svtx.Value(i).ParameterOnLine())
                        {
                            SortIsOK = false;
                            svtx.Exchange(i - 1, i);
                            ExchangeIndices(i);
                        }
                    }
                } while (!SortIsOK);
            }
            SortAgain = false;
            SortIsOK = true;
            for (i = 2; i <= nbvtx && SortIsOK; i++)
            {
                IntPatch_Point VTX = svtx.Value(i);
                for (j = 1; j < i && SortIsOK; j++)
                {
                    IntPatch_Point VTXM1 = svtx.Value(j);
                    bool kill = false;
                    bool killm1 = false;
                    if (Math.Abs(VTXM1.ParameterOnLine() - VTX.ParameterOnLine()) < PCONFUSION)
                    {
                        if (VTXM1.IsOnDomS1() && VTX.IsOnDomS1())
                        { //-- OnS1    OnS1
                            if (ReferenceEquals(VTXM1.ArcOnS1(), VTX.ArcOnS1()))
                            { //-- OnS1 == OnS1
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
                                            kill = true;
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
                        { //-- Pas OnS1  et  OnS1
                            if (!VTXM1.IsOnDomS2() && !VTX.IsOnDomS2())
                            {
                                if (VTXM1.IsOnDomS1() && !VTX.IsOnDomS1())
                                {
                                    kill = true;
                                }
                                else if (VTX.IsOnDomS1() && !VTXM1.IsOnDomS1())
                                {
                                    killm1 = true;
                                }
                            }
                        }

                        if (!(kill || killm1))
                        {
                            if (VTXM1.IsOnDomS2() && VTX.IsOnDomS2())
                            { //-- OnS2    OnS2
                                if (ReferenceEquals(VTXM1.ArcOnS2(), VTX.ArcOnS2()))
                                { //-- OnS2 == OnS2
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
                                                kill = true;
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
                            { //-- Pas OnS2  et  OnS2
                                if (!VTXM1.IsOnDomS1() && !VTX.IsOnDomS1())
                                {
                                    if (VTXM1.IsOnDomS2() && !VTX.IsOnDomS2())
                                    {
                                        kill = true;
                                    }
                                    else if (VTX.IsOnDomS2() && !VTXM1.IsOnDomS2())
                                    {
                                        killm1 = true;
                                    }
                                }
                            }
                        }
                        //-- On a j < i
                        if (kill)
                        {
                            SortIsOK = false;
                            if (lapt)
                            {
                                if (indl > i)
                                {
                                    indl--;
                                }
                                else if (indl == i)
                                {
                                    indl = j;
                                }
                            }
                            if (fipt)
                            {
                                if (indf > i)
                                {
                                    indf--;
                                }
                                else if (indf == i)
                                {
                                    indf = j;
                                }
                            }
                            svtx.Remove(i);
                            nbvtx--;
                        }
                        else if (killm1)
                        {
                            SortIsOK = false;
                            if (lapt)
                            {
                                if (indl > j)
                                {
                                    indl--;
                                }
                                else if (indl == j)
                                {
                                    indl = i - 1;
                                }
                            }
                            if (fipt)
                            {
                                if (indf > j)
                                {
                                    indf--;
                                }
                                else if (indf == j)
                                {
                                    indf = i - 1;
                                }
                            }
                            svtx.Remove(j);
                            nbvtx--;
                        }
                    }
                }
            }
        } while (!SortIsOK);

        //----------------------------------------------------------
        //--   Traitement des lignes periodiques                  --
        //----------------------------------------------------------
        if (!OpenFirst && !OpenLast)
        {
            nbvtx = NbVertex();

            IntPatch_Point VTX0 = svtx.Value(1);
            IntPatch_Point VTXN = svtx.Value(nbvtx);
            if (VTX0.ParameterOnLine() == ParamMinOnLine)
            {
                if (VTXN.ParameterOnLine() != ParamMaxOnLine)
                {
                    gp_Pnt PN = Value(ParamMaxOnLine);
                    double d = PN.Distance(VTX0.Value());
                    if (d <= Tol)
                    {
                        IntPatch_Point OVTX = VTX0;
                        OVTX.SetParameter(ParamMaxOnLine);
                        svtx.Append(OVTX);
                    }
                }
                else
                {
                    // OCCT: unreachable (VTX0 is at ParamMinOnLine in this branch), so a closed line whose last
                    // vertex is at its end never gets the vertex at its start (see occt-issues.md)
                    if (VTXN.ParameterOnLine() == ParamMaxOnLine)
                    {
                        if (VTX0.ParameterOnLine() != ParamMinOnLine)
                        {
                            gp_Pnt P0 = Value(ParamMinOnLine);
                            double d = P0.Distance(VTX0.Value());
                            if (d <= Tol)
                            {
                                IntPatch_Point OVTX = VTXN;
                                OVTX.SetParameter(ParamMinOnLine);
                                svtx.Prepend(OVTX);
                            }
                        }
                    }
                }
            }
        }
        //---------------------------------------------------------
        //-- Faut il supprimer le premier et le dernier point
        //--
        nbvtx = NbVertex();
        if (nbvtx > 1)
        {
            IntPatch_Point VTX0 = svtx.Value(1);
            if ((!VTX0.IsOnDomS1()) && (!VTX0.IsOnDomS2()))
            {
                svtx.Remove(1);
                nbvtx--;
                if (lapt)
                {
                    indl--;
                }
            }
        }
        if (nbvtx > 1)
        {
            IntPatch_Point VTX0 = svtx.Value(nbvtx);
            if ((!VTX0.IsOnDomS1()) && (!VTX0.IsOnDomS2()))
            {
                svtx.Remove(nbvtx);
                if (lapt)
                {
                    indl--;
                }
            }
        }

        //-- Si 2 vertex ont le meme parametre   on identifie le p3d
        nbvtx = NbVertex();
        do
        {
            SortIsOK = true;
            for (i = 2; i <= nbvtx; i++)
            {
                ref IntPatch_Point VTX = ref svtx.ChangeValue(i);
                ref IntPatch_Point VTXm1 = ref svtx.ChangeValue(i - 1);
                if (Math.Abs(VTX.ParameterOnLine() - VTXm1.ParameterOnLine()) < PCONFUSION)
                {
                    if (VTX.IsOnDomS1() && !VTXm1.IsOnDomS1())
                    {
                        VTXm1.SetArc(true, VTX.ArcOnS1(), VTX.ParameterOnArc1(), VTX.TransitionLineArc1(),
                                     VTX.TransitionOnS1());
                    }
                    else if (VTXm1.IsOnDomS1() && !VTX.IsOnDomS1())
                    {
                        VTX.SetArc(true, VTXm1.ArcOnS1(), VTXm1.ParameterOnArc1(), VTXm1.TransitionLineArc1(),
                                   VTXm1.TransitionOnS1());
                    }
                    if (VTX.IsVertexOnS1() && !VTXm1.IsVertexOnS1())
                    {
                        VTXm1.SetVertex(true, VTX.VertexOnS1());
                        VTXm1.SetArc(true, VTX.ArcOnS1(), VTX.ParameterOnArc1(), VTX.TransitionLineArc1(),
                                     VTX.TransitionOnS1());
                    }
                    else if (VTXm1.IsVertexOnS1() && !VTX.IsVertexOnS1())
                    {
                        VTX.SetVertex(true, VTXm1.VertexOnS1());
                        VTX.SetArc(true, VTXm1.ArcOnS1(), VTXm1.ParameterOnArc1(), VTXm1.TransitionLineArc1(),
                                   VTXm1.TransitionOnS1());
                    }

                    if (VTX.IsOnDomS2() && !VTXm1.IsOnDomS2())
                    {
                        VTXm1.SetArc(false, VTX.ArcOnS2(), VTX.ParameterOnArc2(), VTX.TransitionLineArc2(),
                                     VTX.TransitionOnS2());
                    }
                    else if (VTXm1.IsOnDomS2() && !VTX.IsOnDomS2())
                    {
                        VTX.SetArc(false, VTXm1.ArcOnS2(), VTXm1.ParameterOnArc2(), VTXm1.TransitionLineArc2(),
                                   VTXm1.TransitionOnS2());
                    }
                    if (VTX.IsVertexOnS2() && !VTXm1.IsVertexOnS2())
                    {
                        VTXm1.SetVertex(false, VTX.VertexOnS2());
                        VTXm1.SetArc(false, VTX.ArcOnS2(), VTX.ParameterOnArc2(), VTX.TransitionLineArc2(),
                                     VTX.TransitionOnS2());
                    }
                    else if (VTXm1.IsVertexOnS2() && !VTX.IsVertexOnS2())
                    {
                        VTX.SetVertex(false, VTXm1.VertexOnS2());
                        VTX.SetArc(false, VTXm1.ArcOnS2(), VTXm1.ParameterOnArc2(), VTXm1.TransitionLineArc2(),
                                   VTXm1.TransitionOnS2());
                    }

                    if (VTX.Value().SquareDistance(VTXm1.Value()) > 1e-12)
                    {
                        IntPatch_Point CopyVtx = VTXm1;
                        VTXm1.SetParameter(VTX.ParameterOnLine());
                        VTXm1.SetValue(VTX.Value(), VTX.Tolerance(), VTX.IsTangencyPoint());
                        VTX.Parameters(out double u1, out double v1, out double u2, out double v2);
                        VTXm1.SetParameters(u1, v1, u2, v2);
                        if (CopyVtx.IsOnDomS1())
                        {
                            VTXm1.SetArc(true, CopyVtx.ArcOnS1(), CopyVtx.ParameterOnArc1(),
                                         CopyVtx.TransitionLineArc1(), CopyVtx.TransitionOnS1());
                        }
                        if (CopyVtx.IsOnDomS2())
                        {
                            VTXm1.SetArc(false, CopyVtx.ArcOnS2(), CopyVtx.ParameterOnArc2(),
                                         CopyVtx.TransitionLineArc2(), CopyVtx.TransitionOnS2());
                        }
                        if (CopyVtx.IsVertexOnS1())
                        {
                            VTXm1.SetVertex(true, CopyVtx.VertexOnS1());
                            VTXm1.SetArc(true, CopyVtx.ArcOnS1(), CopyVtx.ParameterOnArc1(),
                                         CopyVtx.TransitionLineArc1(), CopyVtx.TransitionOnS1());
                        }
                        if (CopyVtx.IsVertexOnS2())
                        {
                            VTXm1.SetVertex(false, CopyVtx.VertexOnS2());
                            VTXm1.SetArc(false, CopyVtx.ArcOnS2(), CopyVtx.ParameterOnArc2(),
                                         CopyVtx.TransitionLineArc2(), CopyVtx.TransitionOnS2());
                        }

                        SortIsOK = false;
                        //-- std::cout<<" IntPatch_ALine : ComputeVertexParameters : Ajust "<<std::endl;
                    }
                }
            }
        } while (!SortIsOK);
    }

    // The index updates of an exchange of the vertices i - 1 and i, that OCCT writes out in both sorts.
    private void ExchangeIndices(int i)
    {
        if (fipt)
        {
            if (indf == i)
            {
                indf = i - 1;
            }
            else if (indf == i - 1)
            {
                indf = i;
            }
        }
        if (lapt)
        {
            if (indl == i)
            {
                indl = i - 1;
            }
            else if (indl == i - 1)
            {
                indl = i;
            }
        }
    }

    // The removal of a duplicate on a restriction (OCCT's ">=" index updates), written out twice in OCCT.
    private void RemoveDuplicate(int j, ref int nbvtx)
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
    }
}
