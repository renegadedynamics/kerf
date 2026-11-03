// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLine.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using System.Globalization;
using static Occt.IntPatch_IType;

namespace Occt;

/// <summary>
/// An intersection line between two parametric patches defined by a set of points (from a walking algorithm), with
/// its vertices, its first and last points, the periods of the surfaces and the restriction arcs it lies on.
/// </summary>
/// <remarks>
/// A working object (decision 609): the intersection algorithms split, purge and edit it in place.
/// <see cref="Vertex"/> returns a copy; <see cref="ChangeVertex"/> a reference to the stored vertex (valid until the
/// next vertex is added or removed). <see cref="Dump(int, TextWriter)"/> is OCCT's <c>Dump</c> (to stdout). It is
/// the line of GeomInt's instantiations of ApproxInt's generics (<see cref="IApproxInt_WLine{TSelf}"/>).
/// </remarks>
public class IntPatch_WLine : IntPatch_PointLine, IApproxInt_WLine<IntPatch_WLine>
{
    /// <summary>The ways a WLine is created.</summary>
    public enum IntPatch_WLType
    {
        IntPatch_WLUnknown,
        IntPatch_WLImpImp,
        IntPatch_WLImpPrm,
        IntPatch_WLPrmPrm,
    }

    private readonly IntSurf_LineOn2S curv;
    private bool fipt;
    private bool lapt;
    private int indf;
    private int indl;
    private readonly NCollection_Sequence<IntPatch_Point> svtx = new();
    private double u1period;
    private double v1period;
    private double u2period;
    private double v2period;
    private bool hasArcOnS1;
    private Curve2DAdaptorBase? theArcOnS1;
    private bool hasArcOnS2;
    private Curve2DAdaptorBase? theArcOnS2;
    private bool myIsPurgerAllowed;
    // identifies the way of <*this> creation
    private IntPatch_WLType myCreationWay;

    /// <summary>A WLine whose transitions are in or out.</summary>
    public IntPatch_WLine(IntSurf_LineOn2S Line, bool Tang, IntSurf_TypeTrans Trans1, IntSurf_TypeTrans Trans2)
        : base(Tang, Trans1, Trans2)
    {
        fipt = false;
        lapt = false;
        hasArcOnS1 = false;
        hasArcOnS2 = false;
        myIsPurgerAllowed = true;
        myCreationWay = IntPatch_WLType.IntPatch_WLUnknown;
        typ = IntPatch_Walking;
        curv = Line;
        u1period = v1period = u2period = v2period = 0.0;
    }

    /// <summary>A WLine whose transitions are touch.</summary>
    public IntPatch_WLine(IntSurf_LineOn2S Line, bool Tang, IntSurf_Situation Situ1, IntSurf_Situation Situ2)
        : base(Tang, Situ1, Situ2)
    {
        fipt = false;
        lapt = false;
        hasArcOnS1 = false;
        hasArcOnS2 = false;
        myIsPurgerAllowed = true;
        myCreationWay = IntPatch_WLType.IntPatch_WLUnknown;
        typ = IntPatch_Walking;
        curv = Line;
        u1period = v1period = u2period = v2period = 0.0;
    }

    /// <summary>A WLine whose transitions are undecided.</summary>
    public IntPatch_WLine(IntSurf_LineOn2S Line, bool Tang)
        : base(Tang)
    {
        fipt = false;
        lapt = false;
        hasArcOnS1 = false;
        hasArcOnS2 = false;
        myIsPurgerAllowed = true;
        myCreationWay = IntPatch_WLType.IntPatch_WLUnknown;
        typ = IntPatch_Walking;
        curv = Line;
        u1period = v1period = u2period = v2period = 0.0;
    }

    /// <summary>OCCT's <c>new IntPatch_WLine(Line, Tang)</c> in ApproxInt's generics (the TheLine macro).</summary>
    static IntPatch_WLine IApproxInt_WLine<IntPatch_WLine>.Create(IntSurf_LineOn2S lin, bool theTang) => new(lin, theTang);

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

    /// <summary>Sets the point of range <paramref name="Index"/> of the line to the vertex's point and parameters.</summary>
    public void SetPoint(int Index, in IntPatch_Point thepoint) => curv.Value(Index, thepoint.PntOn2S());

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

    public override int NbPnts() => curv.NbPoints();

    public override IntSurf_PntOn2S Point(int Index) => curv.Value(Index);

    /// <summary>True if the line has a known first point (<see cref="FirstPoint()"/>).</summary>
    public bool HasFirstPoint() => fipt;

    /// <summary>True if the line has a known last point (<see cref="LastPoint()"/>).</summary>
    public bool HasLastPoint() => lapt;

    /// <summary>The vertex that is the first point; throws Standard_DomainError if there is none.</summary>
    public IntPatch_Point FirstPoint()
    {
        if (!fipt)
        {
            throw new DomainErrorException();
        }
        return svtx.Value(indf);
    }

    /// <summary>The vertex that is the last point; throws Standard_DomainError if there is none.</summary>
    public IntPatch_Point LastPoint()
    {
        if (!lapt)
        {
            throw new DomainErrorException();
        }
        return svtx.Value(indl);
    }

    /// <summary>The vertex that is the first point, and its index in the list of vertices.</summary>
    public IntPatch_Point FirstPoint(out int Indfirst)
    {
        if (!fipt)
        {
            throw new DomainErrorException();
        }
        Indfirst = indf;
        return svtx.Value(indf);
    }

    /// <summary>The vertex that is the last point, and its index in the list of vertices.</summary>
    public IntPatch_Point LastPoint(out int Indlast)
    {
        if (!lapt)
        {
            throw new DomainErrorException();
        }
        Indlast = indl;
        return svtx.Value(indl);
    }

    public override int NbVertex() => svtx.Length();

    public override IntPatch_Point Vertex(int Index) => svtx.Value(Index);

    public override ref IntPatch_Point ChangeVertex(int Index) => ref svtx[Index];

    public override IntSurf_LineOn2S Curve() => (curv);

    public override bool IsOutSurf1Box(in Point2D theP) => curv.IsOutSurf1Box(theP);

    public override bool IsOutSurf2Box(in Point2D theP) => curv.IsOutSurf2Box(theP);

    public override bool IsOutBox(in Point3D theP) => curv.IsOutBox(theP);

    public void SetPeriod(double pu1, double pv1, double pu2, double pv2)
    {
        u1period = pu1;
        v1period = pv1;
        u2period = pu2;
        v2period = pv2;
    }

    public double U1Period() => (u1period);

    public double V1Period() => (v1period);

    public double U2Period() => (u2period);

    public double V2Period() => (v2period);

    public void SetArcOnS1(Curve2DAdaptorBase A)
    {
        hasArcOnS1 = true;
        theArcOnS1 = A;
    }

    public bool HasArcOnS1() => (hasArcOnS1);

    public Curve2DAdaptorBase? GetArcOnS1() => (theArcOnS1);

    public void SetArcOnS2(Curve2DAdaptorBase A)
    {
        hasArcOnS2 = true;
        theArcOnS2 = A;
    }

    public bool HasArcOnS2() => (hasArcOnS2);

    public Curve2DAdaptorBase? GetArcOnS2() => (theArcOnS2);

    public override void ClearVertexes() => svtx.Clear();

    public override void RemoveVertex(int theIndex)
    {
        if ((theIndex < 1) || (theIndex > NbVertex()))
        {
            throw new OutOfRangeException("Cannot delete not existing vertex");
        }
        svtx.Remove(theIndex);
    }

    /// <summary>Inserts a vertex before <paramref name="theIndex"/> (at least 1), or appends it past the end.</summary>
    public void InsertVertexBefore(int theIndex, in IntPatch_Point thePnt)
    {
        int aNbVertexes = NbVertex();
        int anIndex = Max(theIndex, 1);
        if (anIndex > aNbVertexes)
        {
            svtx.Append(thePnt);
        }
        else
        {
            // OCCT inserts at theIndex, not at the clamped anIndex (a theIndex below 1 throws).
            svtx.InsertBefore(theIndex, thePnt);
        }
    }

    /// <summary>Allows or forbids purging this line.</summary>
    public void EnablePurging(bool theIsEnabled) => myIsPurgerAllowed = theIsEnabled;

    /// <summary>True if purging this line is allowed.</summary>
    public bool IsPurgingAllowed() => myIsPurgerAllowed;

    /// <summary>The way this line was created.</summary>
    public IntPatch_WLType GetCreatingWay() => myCreationWay;

    /// <summary>Sets the way this line was created.</summary>
    public void SetCreatingWayInfo(IntPatch_WLType theAlgo) => myCreationWay = theAlgo;

    private static void RecadreMemePeriode(ref double u1, ref double v1, ref double u2, ref double v2, double anu1,
                                           double anv1, double anu2, double anv2, double U1Period, double V1Period,
                                           double U2Period, double V2Period)
    {
        if (U1Period != 0)
        {
            while (anu1 - u1 > 0.8 * U1Period)
            {
                u1 += U1Period;
            }
            while (u1 - anu1 > 0.8 * U1Period)
            {
                u1 -= U1Period;
            }
        }
        if (U2Period != 0)
        {
            while (anu2 - u2 > 0.8 * U2Period)
            {
                u2 += U2Period;
            }
            while (u2 - anu2 > 0.8 * U2Period)
            {
                u2 -= U2Period;
            }
        }
        if (V1Period != 0)
        {
            while (anv1 - v1 > 0.8 * V1Period)
            {
                v1 += V1Period;
            }
            while (v1 - anv1 > 0.8 * V1Period)
            {
                v1 -= V1Period;
            }
        }
        if (V2Period != 0)
        {
            while (anv2 - v2 > 0.8 * V2Period)
            {
                v2 += V2Period;
            }
            while (v2 - anv2 > 0.8 * V2Period)
            {
                v2 -= V2Period;
            }
        }
    }

    private static void RecadreMemePeriode(ref IntSurf_PntOn2S POn2S, in IntSurf_PntOn2S RefPOn2S, double up1,
                                           double vp1, double up2, double vp2)
    {
        POn2S.Parameters(out double u1, out double v1, out double u2, out double v2);
        RefPOn2S.Parameters(out double pu1, out double pv1, out double pu2, out double pv2);
        RecadreMemePeriode(ref u1, ref v1, ref u2, ref v2, pu1, pv1, pu2, pv2, up1, vp1, up2, vp2);
        POn2S.SetValue(u1, v1, u2, v2);
    }

    private static bool CompareVertexAndPoint(in Point3D V, in Point3D P, double Tol)
    {
        double aSQDist = V.SquareDistance(P);
        double aSQTol = Tol * Tol;
        return (aSQDist <= aSQTol);
    }

    // OCCT's SameVtxRst (a global function of IntPatch_WLine.cxx): the two vertices are on the same arcs at the
    // same parameters, or on none.
    internal static bool SameVtxRst(in IntPatch_Point vtx1, in IntPatch_Point vtx2)
    {
        if (vtx1.IsOnDomS1())
        {
            if (vtx2.IsOnDomS1())
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
            if (vtx2.IsOnDomS1())
            {
                return (false);
            }
        }
        if (vtx1.IsOnDomS2())
        {
            if (vtx2.IsOnDomS2())
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
            if (vtx2.IsOnDomS2())
            {
                return (false);
            }
        }
        return (true);
    }

    private static bool CompareVerticesOnSurf(in IntPatch_Point vtx1, in IntPatch_Point vtx2, bool onFirst)
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

    private static bool CompareVerticesOnS1(in IntPatch_Point vtx1, in IntPatch_Point vtx2) =>
        CompareVerticesOnSurf(vtx1, vtx2, true);

    private static bool CompareVerticesOnS2(in IntPatch_Point vtx1, in IntPatch_Point vtx2) =>
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
    public void ComputeVertexParameters(double RTol)
    {
        // MSV Oct 15, 2001: use tolerance of vertex instead of RTol where
        //                   it is possible

        int i, j, k, nbvtx, nbponline;
        int indicevertexonline;
        double indicevertex;

        bool APointDeleted = false;
        //----------------------------------------------------------
        //--     F i l t r e   s u r   r e s t r i c t i o n s   --
        //----------------------------------------------------------
        //-- deux vertex sur la meme restriction et seulement
        //-- sur celle ci ne doivent pas avoir le meme parametre
        //--
        double Tol = RTol;
        nbvtx = NbVertex();
        // DEBUGV: a dump of the vertices before the computation, compiled out in OCCT: left out.

        //-- ----------------------------------------------------------------------
        //-- Traitement des aretes de couture : On duplique les points situes
        //-- sur des restrictions differentes
        //--
        //-- Phase Creation de nouveaux points sur S1
        bool encoreunefois;
        do
        {
            nbvtx = NbVertex();
            encoreunefois = false;
            for (i = 1; i <= nbvtx && !encoreunefois; i++)
            {
                for (j = 1; j <= nbvtx && !encoreunefois; j++)
                {
                    if (i != j)
                    {
                        if (svtx[i].ParameterOnLine() != svtx[j].ParameterOnLine())
                        {
                            double d = svtx[i].Value().Distance(svtx[j].Value());
                            double toli = svtx[i].Tolerance();
                            double tolj = svtx[j].Tolerance();
                            double maxtol = Max(toli, tolj);
                            // MSV Oct 30, 2001: compare in 2D space also;
                            //                   increase tolerances
                            if (d < maxtol || CompareVerticesOnS1(svtx[i], svtx[j]) || CompareVerticesOnS2(svtx[i], svtx[j]))
                            {
                                //-- Creation Vtx (REF:S1(i)  S2(j))    (On Garde S1(i))
                                double newtoli = Max(toli, tolj + d * 1.01);
                                double newtolj = Max(tolj, toli + d * 1.01);
                                bool acreer = false;
                                if (svtx[i].IsOnDomS1())
                                {
                                    if (svtx[j].IsOnDomS1())
                                    {
                                        if (!ReferenceEquals(svtx[j].ArcOnS1(), svtx[i].ArcOnS1()))
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
                                    IntPatch_Point vtx = svtx[j];
                                    vtx.SetArc(true, svtx[i].ArcOnS1(), svtx[i].ParameterOnArc1(),
                                               svtx[i].TransitionLineArc1(), svtx[i].TransitionOnS1());
                                    for (k = 1; !encoreunefois && k <= nbvtx; k++)
                                    {
                                        if (SameVtxRst(svtx[k], vtx))
                                        {
                                            encoreunefois = true;
                                        }
                                    }
                                    if (!encoreunefois)
                                    {
                                        svtx[i].SetTolerance(newtoli);
                                        svtx[j].SetTolerance(newtolj);
                                        vtx.SetTolerance(newtolj);
                                        svtx.Append(vtx);
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
                                if (svtx[i].IsOnDomS2())
                                {
                                    if (svtx[j].IsOnDomS2())
                                    {
                                        if (!ReferenceEquals(svtx[j].ArcOnS2(), svtx[i].ArcOnS2()))
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
                                    IntPatch_Point vtx = svtx[j];
                                    vtx.SetArc(false, svtx[i].ArcOnS2(), svtx[i].ParameterOnArc2(),
                                               svtx[i].TransitionLineArc2(), svtx[i].TransitionOnS2());
                                    for (k = 1; !encoreunefois && k <= nbvtx; k++)
                                    {
                                        if (SameVtxRst(svtx[k], vtx))
                                        {
                                            encoreunefois = true;
                                        }
                                    }
                                    if (!encoreunefois)
                                    {
                                        svtx[i].SetTolerance(newtoli);
                                        svtx[j].SetTolerance(newtolj);
                                        vtx.SetTolerance(newtolj);
                                        svtx.Append(vtx);
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
            APointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!APointDeleted); i++)
            {
                IntPatch_Point VTXi = svtx.Value(i);
                if (VTXi.Tolerance() > Tol)
                {
                    Tol = VTXi.Tolerance(); //-- 9 oct 97
                }
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
        } while (APointDeleted);

        nbvtx = NbVertex();
        nbponline = NbPnts();

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

        //----------------------------------------------------
        //-- On detecte les points confondus dans la LineOn2S
        double dmini = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        for (i = 2; (i <= nbponline) && (nbponline > 2); i++)
        {
            IntSurf_PntOn2S aPnt1 = curv.Value(i - 1);
            IntSurf_PntOn2S aPnt2 = curv.Value(i);
            double d = (aPnt1.Value()).SquareDistance((aPnt2.Value()));
            if (d < dmini)
            {
                curv.RemovePoint(i);
                nbponline--;
                //----------------------------------------------
                //-- On recadre les Vertex si besoin
                //--
                for (j = 1; j <= nbvtx; j++)
                {
                    indicevertex = svtx.Value(j).ParameterOnLine();
                    if (indicevertex >= i)
                    {
                        svtx[j].SetParameter(indicevertex - 1.0);
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
            Point3D P = svtx.Value(i).Value();
            double vTol = svtx.Value(i).Tolerance();
            indicevertex = svtx.Value(i).ParameterOnLine();
            indicevertexonline = (int)indicevertex;
            //--------------------------------------------------
            //-- On Compare le vertex avec les points de la ligne
            //-- d indice   indicevertexOnLine-1
            //--            indicevertexOnLine
            //--            indicevertexOnLine+1
            //--------------------------------------------------
            if (indicevertexonline < 1)
            {
                if (CompareVertexAndPoint(P, curv.Value(1).Value(), vTol))
                {
                    //-------------------------------------------------------
                    //-- On remplace le point cheminement(1) par vertex(i)
                    //-- et   vertex(i) prend pour parametre 1
                    //-------------------------------------------------------

                    IntSurf_PntOn2S POn2S = svtx.Value(i).PntOn2S();
                    RecadreMemePeriode(ref POn2S, curv.Value(1), U1Period(), V1Period(), U2Period(), V2Period());
                    if (myCreationWay == IntPatch_WLType.IntPatch_WLImpImp)
                    {
                        // Adjust first point of curve to corresponding vertex the following way:
                        // set 3D point as the point of the vertex and 2D points as the points of the point on
                        // curve.
                        curv.SetPoint(1, POn2S.Value());
                        curv.Value(1).Parameters(out double mu1, out double mv1, out double mu2, out double mv2);
                        svtx[i].SetParameter(1);
                        svtx[i].SetParameters(mu1, mv1, mu2, mv2);
                    }
                    else
                    {
                        curv.Value(1, POn2S);
                    }

                    //--curv->Value(1,svtx.Value(i).PntOn2S());
                    svtx[i].SetParameter(1.0);
                }
                else
                {
                    //-------------------------------------------------------
                    //-- On insere le point de cheminement Vertex(i)
                    //-- On recadre les parametres des autres vertex
                    //-------------------------------------------------------
                    IntSurf_PntOn2S POn2S = svtx.Value(i).PntOn2S();
                    RecadreMemePeriode(ref POn2S, curv.Value(1), U1Period(), V1Period(), U2Period(), V2Period());
                    curv.InsertBefore(1, POn2S);

                    //-- curv->InsertBefore(1,svtx.Value(i).PntOn2S());
                    svtx[i].SetParameter(1.0);
                    nbponline++;
                    for (j = 1; j <= nbvtx; j++)
                    {
                        if (j != 1) // OCCT: compares j with 1, not with i (reproduced)
                        {
                            double t = svtx.Value(j).ParameterOnLine();
                            if (t > 1.0)
                            {
                                svtx[j].SetParameter(t + 1.0);
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
                bool Substitution = false;
                //-- for(k=indicevertexonline+1; !Substitution && k>=indicevertexonline-1;k--) {   avant le 9
                // oct 97
                curv.Value(indicevertexonline).Parameters(out double mu1, out double mv1, out double mu2, out double mv2);

                for (k = indicevertexonline + 1; k >= indicevertexonline - 1; k--)
                {
                    if (k > 0 && k <= nbponline)
                    {
                        if (CompareVertexAndPoint(P, curv.Value(k).Value(), vTol))
                        {
                            //-------------------------------------------------------
                            //-- On remplace le point cheminement(k)
                            //-- par vertex(i)  et vertex(i) prend pour parametre k
                            //-------------------------------------------------------
                            IntSurf_PntOn2S POn2S = svtx.Value(i).PntOn2S();
                            RecadreMemePeriode(ref POn2S, curv.Value(k), U1Period(), V1Period(), U2Period(), V2Period());

                            if (myCreationWay == IntPatch_WLType.IntPatch_WLImpImp)
                            {
                                // Adjust a point of curve to corresponding vertex the following way:
                                // set 3D point as the point of the vertex and 2D points as the points
                                // of the point on curve with index <indicevertexonline>
                                curv.SetPoint(k, POn2S.Value());
                                curv.SetUV(k, true, mu1, mv1);
                                curv.SetUV(k, false, mu2, mv2);
                            }
                            else
                            {
                                curv.Value(k, POn2S);
                                POn2S.Parameters(out mu1, out mv1, out mu2, out mv2);
                            }
                            svtx[i].SetParameter(k);
                            svtx[i].SetParameters(mu1, mv1, mu2, mv2);
                            Substitution = true;
                        }
                    }
                }

                // Remove duplicating points
                if (Substitution)
                {
                    int ind_point;
                    for (ind_point = 2; (ind_point <= nbponline && nbponline > 1); ind_point++)
                    {
                        double d = (curv.Value(ind_point - 1).Value()).SquareDistance((curv.Value(ind_point).Value()));
                        if (d < dmini)
                        {
                            curv.RemovePoint(ind_point);
                            nbponline--;
                            //----------------------------------------------
                            //-- On recadre les Vertex si besoin
                            //--
                            for (j = 1; j <= nbvtx; j++)
                            {
                                indicevertex = svtx.Value(j).ParameterOnLine();
                                if (indicevertex >= ind_point)
                                {
                                    svtx[j].SetParameter(indicevertex - 1.0);
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

                if (!Substitution)
                {
                    //-------------------------------------------------------
                    //-- On insere le point de cheminement Vertex(i)
                    //-- On recadre les parametres des autres vertex
                    //-------------------------------------------------------
                    IntSurf_PntOn2S POn2S = svtx.Value(i).PntOn2S();
                    if (indicevertexonline >= nbponline)
                    {
                        RecadreMemePeriode(ref POn2S, curv.Value(nbponline), U1Period(), V1Period(), U2Period(),
                                           V2Period());
                        curv.Add(POn2S);
                    }
                    else
                    {
                        RecadreMemePeriode(ref POn2S, curv.Value(indicevertexonline + 1), U1Period(), V1Period(),
                                           U2Period(), V2Period());
                        curv.InsertBefore(indicevertexonline + 1, POn2S);
                    }
                    //-- curv->InsertBefore(indicevertexonline+1,svtx.Value(i).PntOn2S());
                    svtx[i].SetParameter(indicevertexonline + 1);
                    nbponline++;
                    for (j = 1; j <= nbvtx; j++)
                    {
                        if (j != i)
                        {
                            double t = svtx.Value(j).ParameterOnLine();
                            if (t > (double)indicevertexonline)
                            {
                                svtx[j].SetParameter(t + 1.0);
                            }
                        }
                    }
                } //-- Substitution
            } //-- indicevertexonline>=1

        } //-- boucle i sur vertex

        do
        {
            APointDeleted = false;
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
                            }
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
            double pol = svtx.Value(i).ParameterOnLine();
            if (pol == 1.0)
            {
                bFirst = fipt = true;
                indf = i;
            }
            if (pol == nbponline)
            {
                bLast = lapt = true;
                indl = i;
            }
        }
        if (!bFirst)
        {
            bool vtxfound = false;
            IntPatch_Point vtx = new();
            curv.Value(1).Parameters(out double pu1, out double pv1, out double pu2, out double pv2);

            for (i = 1; (!vtxfound) && (i <= nbvtx); i++)
            {
                IntPatch_Point V = svtx.Value(i);
                // jgv: to avoid loops
                // double vTol = V.Tolerance();
                if (CompareVertexAndPoint(V.Value(), curv.Value(1).Value(), global::Kerf.Engine.Foundation.Precision.Confusion() /*vTol*/))
                {
                    vtx = V;
                    vtx.SetParameters(pu1, pv1, pu2, pv2);
                    vtxfound = true;
                }
            }
            if (!vtxfound)
            {
                vtx.SetValue(curv.Value(1).Value(), Tol, false);
                vtx.SetParameters(pu1, pv1, pu2, pv2);
            }
            vtx.SetParameter(1);
            svtx.Prepend(vtx);
            nbvtx++;
            fipt = true;
            indf = 1;
        }
        if (!bLast)
        {
            bool vtxfound = false;
            IntPatch_Point vtx = new();
            curv.Value(nbponline).Parameters(out double pu1, out double pv1, out double pu2, out double pv2);

            for (i = 1; (!vtxfound) && (i <= nbvtx); i++)
            {
                IntPatch_Point V = svtx.Value(i);
                // jgv: to avoid loops
                // double vTol = V.Tolerance();
                if (CompareVertexAndPoint(V.Value(), curv.Value(nbponline).Value(), global::Kerf.Engine.Foundation.Precision.Confusion() /*vTol*/))
                {
                    vtx = V;
                    vtx.SetParameters(pu1, pv1, pu2, pv2);
                    vtxfound = true;
                }
            }
            if (!vtxfound)
            {
                vtx.SetValue(curv.Value(nbponline).Value(), Tol, false);
                vtx.SetParameters(pu1, pv1, pu2, pv2);
            }
            vtx.SetParameter(nbponline);
            svtx.Append(vtx);
            nbvtx++;
            lapt = true;
            indl = nbvtx;
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
                                if ((VTXi.ParameterOnLine() == VTXj.ParameterOnLine() + 1)
                                    || (VTXi.ParameterOnLine() == VTXj.ParameterOnLine() - 1))
                                {
                                    if (ReferenceEquals(VTXi.ArcOnS1(), VTXj.ArcOnS1()))
                                    {
                                        IntSurf_Transition t1 = VTXi.TransitionLineArc1();
                                        IntSurf_Transition t2 = VTXj.TransitionLineArc1();
                                        if (t1.TransitionType() == t2.TransitionType())
                                        {
                                            if ((VTXj.ParameterOnLine() != 1) && (VTXj.ParameterOnLine() != NbPnts()))
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
            }
        }
        //-- meme traitement sur les restrictions du second shape
        while (APointDeleted);

        nbvtx = NbVertex();
        do
        {
            APointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!APointDeleted); i++)
            {
                IntPatch_Point VTXi = svtx.Value(i);
                if ((!VTXi.IsOnDomS1()) && (VTXi.IsOnDomS2()))
                {
                    for (j = 1; (j <= nbvtx) && (!APointDeleted); j++)
                    {
                        if (i != j)
                        {
                            IntPatch_Point VTXj = svtx.Value(j);
                            if ((!VTXj.IsOnDomS1()) && (VTXj.IsOnDomS2()))
                            {
                                if ((VTXi.ParameterOnLine() == VTXj.ParameterOnLine() + 1)
                                    || (VTXi.ParameterOnLine() == VTXj.ParameterOnLine() - 1))
                                {
                                    if (ReferenceEquals(VTXi.ArcOnS2(), VTXj.ArcOnS2()))
                                    {
                                        IntSurf_Transition t1 = VTXi.TransitionLineArc2();
                                        IntSurf_Transition t2 = VTXj.TransitionLineArc2();
                                        if (t1.TransitionType() == t2.TransitionType())
                                        {
                                            if ((VTXj.ParameterOnLine() != 1) && (VTXj.ParameterOnLine() != NbPnts()))
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
            }
        } while (APointDeleted);
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

        // DEBUGV: a dump of the vertices after the computation, compiled out in OCCT: left out.
    }

    /// <summary>
    /// Writes the line as OCCT's <c>Dump</c> prints it: mode 0 the points and vertices, 1 the 3D points, 2 the points
    /// on the first surface, others the points on the second surface.
    /// </summary>
    public void Dump(int theMode, TextWriter theOut) => DumpPointLine(this, " W", theMode, theOut);

    // The body of WLine's and RLine's Dump, which differ only in the title.
    internal static void DumpPointLine(IntPatch_PointLine theLine, string theTitle, int theMode, TextWriter theOut)
    {
        theOut.Write(" ----------- D u m p    I n t P a t c h  _ " + theTitle + " L i n e  -(begin)------" + '\n');
        int aNbPoints = theLine.NbPnts();
        int aNbVertex = theLine.NbVertex();

        switch (theMode)
        {
            case 0:
                theOut.Write("Num    [X  Y  Z]     [U1  V1]   [U2  V2]\n");
                for (int i = 1; i <= aNbPoints; i++)
                {
                    IntSurf_PntOn2S aP = theLine.Point(i);
                    aP.Parameters(out double u1, out double v1, out double u2, out double v2);
                    theOut.Write($"{i.ToString(CultureInfo.InvariantCulture),4}  [{F(aP.Value().X())} {F(aP.Value().Y())} "
                                 + $"{F(aP.Value().Z())}]  [{F(u1)} {F(v1)}]  [{F(u2)} {F(v2)}]\n");
                }

                for (int i = 1; i <= aNbVertex; i++)
                {
                    theLine.Vertex(i).Dump(theOut);
                    double polr = theLine.Vertex(i).ParameterOnLine();
                    int pol = (int)polr;

                    // OCCT: indexes the vertices with the point's rank on the line (Vertex(pol), not Point(pol)),
                    // and so only when that rank is at most the number of vertices (reproduced).
                    if (pol >= 1 && pol <= aNbVertex)
                    {
                        Point3D aV = theLine.Vertex(pol).Value();
                        theOut.Write("----> IntSurf_PntOn2S : " + R(polr) + ", Pnt (" + R(aV.X()) + "," + R(aV.Y())
                                     + "," + R(aV.Z()) + ")" + '\n');
                    }
                }
                break;
            case 1:
                for (int i = 1; i <= aNbPoints; i++)
                {
                    Point3D aP = theLine.Point(i).Value();
                    theOut.Write($"point p{i.ToString(CultureInfo.InvariantCulture)} {F(aP.X())} {F(aP.Y())} {F(aP.Z())}\n");
                }
                break;
            case 2:
                for (int i = 1; i <= aNbPoints; i++)
                {
                    theLine.Point(i).Parameters(out double u1, out double v1, out _, out _);
                    theOut.Write($"point p{i.ToString(CultureInfo.InvariantCulture)} {F(u1)} {F(v1)}\n");
                }
                break;
            default:
                for (int i = 1; i <= aNbPoints; i++)
                {
                    theLine.Point(i).Parameters(out _, out _, out double u2, out double v2);
                    theOut.Write($"point p{i.ToString(CultureInfo.InvariantCulture)} {F(u2)} {F(v2)}\n");
                }
                break;
        }
        theOut.Write("\n--------------------------------------------------- (end) -------" + '\n');
    }

    // printf's "%+10.20f": a sign always, twenty decimals.
    private static string F(double theValue)
    {
        string aStr = theValue.ToString("F20", CultureInfo.InvariantCulture);
        return aStr.StartsWith('-') ? aStr : "+" + aStr;
    }

    // std::cout << double: six significant digits.
    private static string R(double theValue) => StreamUtilities.FormatReal(theValue, 6);
}
