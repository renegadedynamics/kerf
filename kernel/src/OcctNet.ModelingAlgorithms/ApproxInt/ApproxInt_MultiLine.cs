// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_MultiLine.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Geometry;
using System.Globalization;

namespace Occt;

/// <summary>
/// What ApproxInt's generics read from a walking line (OCCT's TheLine and TheWLine macros: IntPatch_WLine in GeomInt,
/// BRepApprox_ApproxLine in BRepApprox): its points, and a new line over a sequence of points.
/// </summary>
/// <remarks>Not an OCCT type (decision 607's instance interfaces).</remarks>
/// <typeparam name="TSelf">The line type.</typeparam>
public interface IApproxInt_WLine<TSelf> where TSelf : class, IApproxInt_WLine<TSelf>
{
    /// <summary>The number of points of the line.</summary>
    int NbPnts();

    /// <summary>The point <paramref name="Index"/> (from 1).</summary>
    IntSurf_PntOn2S Point(int Index);

    /// <summary>OCCT's <c>new TheLine(lin, theTang)</c>: a line on the points of <paramref name="lin"/>.</summary>
    static abstract TSelf Create(IntSurf_LineOn2S lin, bool theTang);
}

/// <summary>
/// The MultiLine of ApproxInt's approximation: the points <c>indicemin</c> to <c>indicemax</c> of a walking line as
/// one 3D point (translated by Xo, Yo, Zo) and up to two 2D points (translated by U1o, V1o and U2o, V2o) each, with
/// tangents and new points computed on the surfaces through an <see cref="ApproxInt_SvSurfaces"/>.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's generic file ApproxInt_MultiLine.gxx, ported once as a generic class over the line (decision 607). TheSvSurfaces
/// is ApproxInt_SvSurfaces in every instantiation, so the body names it; OCCT's untyped pointer to it is a typed
/// reference (null: no points are ever added). OCCT's instantiations (GeomInt_TheMultiLineOfWLApprox,
/// BRepApprox_TheMultiLineOfApprox) are sealed subclasses; the generic algorithms build this class.
/// </para>
/// <para>
/// A C++ value class whose state never changes after construction: a class with read-only fields, as the MultiLine
/// argument of the AppParCurves and Approx generics expects (decision 142). It shares the line and the SvSurfaces it
/// is given, as OCCT's copies do.
/// </para>
/// </remarks>
/// <typeparam name="TheLine">The walking line.</typeparam>
public class ApproxInt_MultiLine<TheLine> where TheLine : class, IApproxInt_WLine<TheLine>
{
    private readonly ApproxInt_SvSurfaces? PtrOnmySvSurfaces;
    private readonly TheLine? myLine;
    private readonly int indicemin;
    private readonly int indicemax;
    private readonly int nbp3d;
    private readonly int nbp2d;
    private readonly bool myApproxU1V1;
    private readonly bool myApproxU2V2;
    private readonly bool p2donfirst;
    private readonly double Xo;
    private readonly double Yo;
    private readonly double Zo;
    private readonly double U1o;
    private readonly double V1o;
    private readonly double U2o;
    private readonly double V2o;

    /// <summary>An empty MultiLine, without a line.</summary>
    public ApproxInt_MultiLine()
    {
        PtrOnmySvSurfaces = null;
        myLine = null;
        indicemin = 0;
        indicemax = 0;
        nbp3d = 0;
        nbp2d = 0;
        myApproxU1V1 = false;
        myApproxU2V2 = false;
        p2donfirst = true;
        Xo = 0.0;
        Yo = 0.0;
        Zo = 0.0;
        U1o = 0.0;
        V1o = 0.0;
        U2o = 0.0;
        V2o = 0.0;
    }

    /// <summary>
    /// The points <paramref name="IndMin"/> to <paramref name="IndMax"/> of <paramref name="line"/>.
    /// <paramref name="svsurf"/> computes the tangents and the new points when the approximation needs some extra
    /// points: a new line is then made, which shares the same surfaces and functions.
    /// </summary>
    public ApproxInt_MultiLine(TheLine line, ApproxInt_SvSurfaces? svsurf, int NbP3d, int NbP2d, bool ApproxU1V1,
                               bool ApproxU2V2, double xo, double yo, double zo, double u1o, double v1o, double u2o,
                               double v2o, bool P2DOnFirst, int IndMin = 0, int IndMax = 0)
    {
        PtrOnmySvSurfaces = svsurf;
        myLine = line;
        indicemin = Min(IndMin, IndMax);
        indicemax = Max(IndMin, IndMax);
        nbp3d = NbP3d;
        nbp2d = NbP2d;
        myApproxU1V1 = ApproxU1V1;
        myApproxU2V2 = ApproxU2V2;
        p2donfirst = P2DOnFirst;
        Xo = xo;
        Yo = yo;
        Zo = zo;
        U1o = u1o;
        V1o = v1o;
        U2o = u2o;
        V2o = v2o;
    }

    /// <summary>As the constructor above without SvSurfaces: no extra points will be added on the line.</summary>
    public ApproxInt_MultiLine(TheLine line, int NbP3d, int NbP2d, bool ApproxU1V1, bool ApproxU2V2, double xo,
                               double yo, double zo, double u1o, double v1o, double u2o, double v2o, bool P2DOnFirst,
                               int IndMin = 0, int IndMax = 0)
        : this(line, null, NbP3d, NbP2d, ApproxU1V1, ApproxU2V2, xo, yo, zo, u1o, v1o, u2o, v2o, P2DOnFirst, IndMin,
               IndMax)
    {
    }

    public int FirstPoint() => indicemin;

    public int LastPoint() => indicemax;

    /// <summary>Approx_PointsAdded when points can be added (there are SvSurfaces), else Approx_NoPointsAdded.</summary>
    public Status WhatStatus()
    {
        if (PtrOnmySvSurfaces is not null)
        {
            return Status.PointsAdded;
        }
        else
        {
            return Status.NoPointsAdded;
        }
    }

    /// <summary>The number of 3d points of a MultiPoint.</summary>
    public int NbP3d() => nbp3d;

    /// <summary>The number of 2d points of a MultiPoint.</summary>
    public int NbP2d() => nbp2d;

    /// <summary>The 3d point of the MultiPoint <paramref name="Index"/>.</summary>
    public void Value(int Index, NCollection_Array1<Point3D> TabPnt)
    {
        Point3D aP = myLine!.Point(Index).Value();
        TabPnt[1].SetCoord(aP.X() + Xo, aP.Y() + Yo, aP.Z() + Zo);
    }

    /// <summary>The 2d points of the MultiPoint <paramref name="Index"/>.</summary>
    public void Value(int Index, NCollection_Array1<Point2D> TabPnt2d)
    {
        IntSurf_PntOn2S POn2S = myLine!.Point(Index);
        POn2S.Parameters(out double u1, out double v1, out double u2, out double v2);
        if (nbp2d == 1)
        {
            if (p2donfirst)
            {
                TabPnt2d[1].SetCoord(u1 + U1o, v1 + V1o);
            }
            else
            {
                TabPnt2d[1].SetCoord(u2 + U2o, v2 + V2o);
            }
        }
        else
        {
            TabPnt2d[1].SetCoord(u1 + U1o, v1 + V1o);
            if (TabPnt2d.Length() >= 2)
            {
                TabPnt2d[2].SetCoord(u2 + U2o, v2 + V2o);
            }
        }
    }

    /// <summary>The 3d and 2d points of the MultiPoint <paramref name="Index"/>.</summary>
    public void Value(int Index, NCollection_Array1<Point3D> TabPnt, NCollection_Array1<Point2D> TabPnt2d)
    {
        Value(Index, TabPnt);
        Value(Index, TabPnt2d);
    }

    /// <summary>The 3d tangent at the MultiPoint <paramref name="Index"/>; false (a null vector) where there is none.</summary>
    public bool Tangency(int Index, NCollection_Array1<Vector3D> TabVec)
    {
        if (PtrOnmySvSurfaces is null)
        {
            return false;
        }

        IntSurf_PntOn2S POn2S = myLine!.Point(Index);
        POn2S.Parameters(out double u1, out double v1, out double u2, out double v2);

        bool ret = PtrOnmySvSurfaces.Tangency(u1, v1, u2, v2, ref TabVec[1]);
        if (!ret)
        {
            TabVec[1].SetCoord(0.0, 0.0, 0.0);
        }

        return ret;
    }

    /// <summary>The 2d tangents at the MultiPoint <paramref name="Index"/>; false (null vectors) where there are none.</summary>
    public bool Tangency(int Index, NCollection_Array1<Vector2D> TabVec2d)
    {
        if (PtrOnmySvSurfaces is null)
        {
            return false;
        }

        IntSurf_PntOn2S POn2S = myLine!.Point(Index);
        POn2S.Parameters(out double u1, out double v1, out double u2, out double v2);

        bool ret = false;
        if (nbp2d == 1)
        {
            if (p2donfirst)
            {
                ret = PtrOnmySvSurfaces.TangencyOnSurf1(u1, v1, u2, v2, ref TabVec2d[1]);
            }
            else
            {
                ret = PtrOnmySvSurfaces.TangencyOnSurf2(u1, v1, u2, v2, ref TabVec2d[1]);
            }
        }
        else
        {
            ret = PtrOnmySvSurfaces.TangencyOnSurf1(u1, v1, u2, v2, ref TabVec2d[1]);
            if (ret)
            {
                if (TabVec2d.Length() >= 2)
                {
                    ret = (ret && PtrOnmySvSurfaces.TangencyOnSurf2(u1, v1, u2, v2, ref TabVec2d[2]));
                }
            }
        }

        if (!ret)
        {
            TabVec2d[1] = new Vector2D(0.0, 0.0);
            if (TabVec2d.Length() >= 2)
            {
                TabVec2d[2] = new Vector2D(0.0, 0.0);
            }
        }

        return ret;
    }

    /// <summary>The 3d and 2d tangents at the MultiPoint <paramref name="Index"/>.</summary>
    public bool Tangency(int Index, NCollection_Array1<Vector3D> TabVec, NCollection_Array1<Vector2D> TabVec2d)
    {
        return (Tangency(Index, TabVec) && Tangency(Index, TabVec2d));
    }

    /// <summary>
    /// Tries to make a sub-line between the points <paramref name="Low"/> and <paramref name="High"/> of this line
    /// by adding <paramref name="aNbPntsToInsert"/> new points; an empty line if it can't.
    /// </summary>
    public ApproxInt_MultiLine<TheLine> MakeMLBetween(int Low, int High, int aNbPntsToInsert)
    {
        if (PtrOnmySvSurfaces is null)
        {
            //-- cout<<"\n Erreur dans : ApproxInt_MultiLine  ApproxInt_MultiLine::MakeMLBetween "<<endl;
            IntSurf_LineOn2S vide1 = new();
            TheLine vide = TheLine.Create(vide1, false);
            return (new ApproxInt_MultiLine<TheLine>(vide, null, nbp3d, nbp2d, myApproxU1V1, myApproxU2V2, Xo, Yo, Zo,
                                                     U1o, V1o, U2o, V2o, p2donfirst, 1, 1));
            //-- return(*this);
        }

        bool aSaveUseSolver = PtrOnmySvSurfaces.GetUseSolver();
        if (!aSaveUseSolver)
        {
            PtrOnmySvSurfaces.SetUseSolver(true);
        }
        int NbPntsToInsert = aNbPntsToInsert;
        if (NbPntsToInsert < (High - Low))
        {
            NbPntsToInsert = (High - Low);
        }
        int NbPnts = NbPntsToInsert + High - Low + 1;
        int NbPntsmin = High - Low;
        NbPntsmin += NbPntsmin;

        if (NbPnts < NbPntsmin)
        {
            NbPnts = NbPntsmin;
        }

        Vector3D T = new();
        Vector2D TS1 = new(), TS2 = new();
        Point3D P = new();

        //-----------------------l-------------------------------------------
        //--  Indice     :   Low       Low+1     I    I+1         High    --
        //--                                                              --
        //--  Abs.Curv.  :  S(Low)              S(I)  S(I+1)      S(High) --
        //--                                                              --
        //--                Sampled at constant curvilinear abscissa      --
        //--                (computed on the 3D points)                   --
        //------------------------------------------------------------------
        NCollection_Array1<double> U1 = new(Low, High);
        NCollection_Array1<double> V1 = new(Low, High);
        NCollection_Array1<double> U2 = new(Low, High);
        NCollection_Array1<double> V2 = new(Low, High);
        NCollection_Array1<double> AC = new(Low, High);
        double s, ds;

        //------------------------------------------------------------
        //-- The arrays U1 .. V2 and AC
        //--
        double u1, v1, u2, v2;
        int i;
        myLine!.Point(Low).Parameters(out u1, out v1, out u2, out v2);
        U1[Low] = u1;
        V1[Low] = v1;
        U2[Low] = u2;
        V2[Low] = v2;
        AC[Low] = 0.0;

        //-- Trial of 19 June 96 (parametrisation by curvilinear abscissa in XYZ)
        for (i = Low + 1; i <= High; i++)
        {
            myLine.Point(i).Parameters(out u1, out v1, out u2, out v2);
            U1[i] = u1;
            V1[i] = v1;
            U2[i] = u2;
            V2[i] = v2;
            AC[i] = AC[i - 1] + (myLine.Point(i - 1).Value()).Distance(myLine.Point(i).Value());
        }

        //-------------------------------------------------------------
        //-- The structures holding the results

        IntSurf_LineOn2S ResultPntOn2SLine = new();

        IntSurf_PntOn2S StartPOn2S = new();

        ds = AC[High] / (NbPnts - 1);
        int Indice = Low;
        bool HasBeenInserted = false;
        double dsmin = ds * 0.3;
        double smax = AC[High];

        for (i = 2, s = ds; (s < smax && Indice <= High - 1); i++, s += ds)
        {
            //----------------------------------------------------------
            //-- Search of the indices of the points                  --
            //-- Point           :    2       i       NbPnts-1        --
            //-- s                            s                       --
            //-- Current Indice such that AC(Indice)<= s < AC(Indice+1)--
            //----------------------------------------------------------
            while (AC[Indice + 1] <= s)
            {
                if (!HasBeenInserted)
                {
                    ResultPntOn2SLine.Add(myLine.Point(Indice));
                }

                HasBeenInserted = false;
                Indice++;
                if (Indice == High)
                {
                    break;
                }
            }

            if (Indice == High)
            {
                break;
            }

            if (!HasBeenInserted && AC[Indice] <= s)
            {
                ResultPntOn2SLine.Add(myLine.Point(Indice));
                HasBeenInserted = true;
            }

            double a = s - AC[Indice];
            double b = AC[Indice + 1] - s;
            double nab = 1.0 / (a + b);

            //----------------------------------------------------------
            //-- Check: distance to the next point < dsmin            --
            //--        distance to the previous point < dsmin        --
            //----------------------------------------------------------
            if ((a > dsmin) && (b > dsmin))
            {
                u1 = (U1[Indice] * b + U1[Indice + 1] * a) * nab;
                v1 = (V1[Indice] * b + V1[Indice + 1] * a) * nab;
                u2 = (U2[Indice] * b + U2[Indice + 1] * a) * nab;
                v2 = (V2[Indice] * b + V2[Indice + 1] * a) * nab;

                if (PtrOnmySvSurfaces.Compute(ref u1, ref v1, ref u2, ref v2, ref P, ref T, ref TS1, ref TS2))
                {
                    StartPOn2S.SetValue(P, u1, v1, u2, v2);
                    ResultPntOn2SLine.Add(StartPOn2S);
                }
            }
            else
            {
                //-- Point not far enough from 2 existing points
                //-- with the point p[indice] already inserted

                if (b < 0.0)
                {
                    while (AC[Indice + 1] <= s)
                    {
                        if (!HasBeenInserted)
                        {
                            ResultPntOn2SLine.Add(myLine.Point(Indice));
                        }

                        HasBeenInserted = false;
                        Indice++;

                        if (Indice == High)
                        {
                            break;
                        }
                    }

                    if (Indice == High)
                    {
                        break;
                    }

                    if (!HasBeenInserted && AC[Indice] <= s)
                    {
                        ResultPntOn2SLine.Add(myLine.Point(Indice));
                        HasBeenInserted = true;
                    }
                }
                else
                {
                    s += (dsmin - ds);
                }
            }
        }

        ResultPntOn2SLine.Add(myLine.Point(High)); //--  Point NbPnts
        TheLine temp = TheLine.Create(ResultPntOn2SLine, false);

        //-- A posteriori check
        //-- There must be no too sharp turn in 2d and in 3d

        temp.Point(1).Parameters(out u1, out v1, out u2, out v2);
        Point2D P1A = new(u1, v1);
        Point2D P2A = new(u2, v2);

        temp.Point(2).Parameters(out u1, out v1, out u2, out v2);
        Point2D P1B = new(u1, v1);
        Point2D P2B = new(u2, v2);

        Point2D P1C = new(), P2C = new();

        int CodeErreur = 0;

        for (i = 3, NbPnts = temp.NbPnts(); CodeErreur == 0 && i <= NbPnts; i++)
        {
            double d, du, dv, duv2;
            temp.Point(i).Parameters(out u1, out v1, out u2, out v2);
            //-- Turn P1A P1B P1C
            P1C.SetCoord(u1, v1);
            du = P1B.X() - P1A.X();
            dv = P1B.Y() - P1A.Y();
            duv2 = 0.25 * (du * du + dv * dv);
            u1 = P1B.X() + du;
            v1 = P1B.Y() + dv;
            du = P1C.X() - u1;
            dv = P1C.Y() - v1;
            d = du * du + dv * dv;
            if (d > duv2)
            {
                CodeErreur = 1;
                break;
            }

            //-- Turn P2A P2B P2C
            P2C.SetCoord(u2, v2);
            du = P2B.X() - P2A.X();
            dv = P2B.Y() - P2A.Y();
            duv2 = 0.25 * (du * du + dv * dv);
            u2 = P2B.X() + du;
            v2 = P2B.Y() + dv;
            du = P2C.X() - u2;
            dv = P2C.Y() - v2;
            d = du * du + dv * dv;
            if (d > duv2)
            {
                CodeErreur = 2;
                break;
            }

            P1A = P1B;
            P2A = P2B;
            P1B = P1C;
            P2B = P2C;
        }

        if ((temp.NbPnts() >= NbPntsToInsert + High - Low + 1) && (CodeErreur == 0))
        {
            PtrOnmySvSurfaces.SetUseSolver(aSaveUseSolver);
            return (new ApproxInt_MultiLine<TheLine>(temp, (High - Low > 10) ? PtrOnmySvSurfaces : null, nbp3d, nbp2d,
                                                     myApproxU1V1, myApproxU2V2, Xo, Yo, Zo, U1o, V1o, U2o, V2o,
                                                     p2donfirst, 1, ResultPntOn2SLine.NbPoints()));
        }
        else
        {
            PtrOnmySvSurfaces.SetUseSolver(aSaveUseSolver);
            IntSurf_LineOn2S vide1 = new();
            TheLine vide = TheLine.Create(vide1, false);
            return (new ApproxInt_MultiLine<TheLine>(vide, null, nbp3d, nbp2d, myApproxU1V1, myApproxU2V2, Xo, Yo, Zo,
                                                     U1o, V1o, U2o, V2o, p2donfirst, 1, 1));
        }
    }

    /// <summary>
    /// Tries to make a sub-line between the points <paramref name="theLow"/> and <paramref name="theHigh"/> of this
    /// line by adding one more point between the points (<paramref name="theIndbad"/> - 1) and
    /// <paramref name="theIndbad"/>.
    /// </summary>
    /// <remarks>
    /// Where it returns false, OCCT leaves <paramref name="theNewMultiLine"/> unchanged (its callers pass a
    /// default-constructed line): it is an empty MultiLine (decision 202).
    /// </remarks>
    public bool MakeMLOneMorePoint(int theLow, int theHigh, int theIndbad,
                                   out ApproxInt_MultiLine<TheLine> theNewMultiLine)
    {
        theNewMultiLine = new ApproxInt_MultiLine<TheLine>();
        bool OtherLineMade = false;
        if (PtrOnmySvSurfaces is null)
        {
            return false;
        }

        bool aSaveUseSolver = PtrOnmySvSurfaces.GetUseSolver();
        if (!aSaveUseSolver)
        {
            PtrOnmySvSurfaces.SetUseSolver(true);
        }

        double SqTol3d = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = tolerance[2] = 1.0e-8;

        IntSurf_LineOn2S ResultPntOn2SLine = new();
        for (int Indice = theLow; Indice <= theHigh; Indice++)
        {
            ResultPntOn2SLine.Add(myLine!.Point(Indice));
        }

        // Insert new point between (theIndbad-1) and theIndbad
        // Using <thePtrSVSurf> for Rsnld: it may be ImpPrm or PrmPrm
        Point3D PrevPnt = myLine!.Point(theIndbad - 1).Value();
        Point3D CurPnt = myLine.Point(theIndbad).Value();
        myLine.Point(theIndbad - 1).Parameters(out double uprev1, out double vprev1, out double uprev2, out double vprev2);
        myLine.Point(theIndbad).Parameters(out double ucur1, out double vcur1, out double ucur2, out double vcur2);
        double umid1, vmid1, umid2, vmid2;
        umid1 = (uprev1 + ucur1) / 2;
        vmid1 = (vprev1 + vcur1) / 2;
        umid2 = (uprev2 + ucur2) / 2;
        vmid2 = (vprev2 + vcur2) / 2;
        IntSurf_PntOn2S MidPoint = new();
        bool IsNewPointInvalid = false;
        IsNewPointInvalid = myApproxU1V1 && Math.Abs(ucur1 - umid1) <= tolerance[1]
                            && Math.Abs(vcur1 - vmid1) <= tolerance[2];
        if (!IsNewPointInvalid)
        {
            IsNewPointInvalid = myApproxU2V2 && Math.Abs(ucur2 - umid2) <= tolerance[1]
                                && Math.Abs(vcur2 - vmid2) <= tolerance[2];
            if (!IsNewPointInvalid && PtrOnmySvSurfaces.SeekPoint(umid1, vmid1, umid2, vmid2, ref MidPoint))
            {
                Point3D NewPnt = MidPoint.Value();
                double SqDistNewPrev = NewPnt.SquareDistance(PrevPnt);
                double SqDistNewCur = NewPnt.SquareDistance(CurPnt);
                IsNewPointInvalid = (SqDistNewPrev <= SqTol3d || SqDistNewCur <= SqTol3d);
                if (!IsNewPointInvalid)
                {
                    MidPoint.Parameters(out double unew1, out double vnew1, out double unew2, out double vnew2);
                    if (myApproxU1V1)
                    {
                        double SqDistCurMid1 = (ucur1 - umid1) * (ucur1 - umid1) + (vcur1 - vmid1) * (vcur1 - vmid1);
                        double SqDistMidNew1 = (umid1 - unew1) * (umid1 - unew1) + (vmid1 - vnew1) * (vmid1 - vnew1);
                        IsNewPointInvalid = (SqDistMidNew1 > SqDistCurMid1);
                    }
                    if (!IsNewPointInvalid)
                    {
                        if (myApproxU2V2)
                        {
                            double SqDistCurMid2 =
                                (ucur2 - umid2) * (ucur2 - umid2) + (vcur2 - vmid2) * (vcur2 - vmid2);
                            double SqDistMidNew2 =
                                (umid2 - unew2) * (umid2 - unew2) + (vmid2 - vnew2) * (vmid2 - vnew2);
                            IsNewPointInvalid = (SqDistMidNew2 > SqDistCurMid2);
                        }
                        if (!IsNewPointInvalid)
                        {
                            ResultPntOn2SLine.InsertBefore(theIndbad - theLow + 1, MidPoint);
                            OtherLineMade = true;
                        }
                    }
                }
            }
        }

        if (!OtherLineMade)
        {
            PtrOnmySvSurfaces.SetUseSolver(aSaveUseSolver);
            return false;
        }

        TheLine temp = TheLine.Create(ResultPntOn2SLine, false);
        PtrOnmySvSurfaces.SetUseSolver(aSaveUseSolver);
        theNewMultiLine = new ApproxInt_MultiLine<TheLine>(temp, PtrOnmySvSurfaces, nbp3d, nbp2d, myApproxU1V1,
                                                           myApproxU2V2, Xo, Yo, Zo, U1o, V1o, U2o, V2o, p2donfirst,
                                                           1, ResultPntOn2SLine.NbPoints());
        return true;
    }

    /// <summary>Writes the points of the MultiLine, one per line, as OCCT prints them on the standard output.</summary>
    public void Dump(TextWriter theOut)
    {
        NCollection_Array1<Point3D> anArr1 = new(1, 1);
        NCollection_Array1<Point2D> anArr2 = new(1, 2);

        int anIndF = FirstPoint(), anIndL = LastPoint();

        for (int ind = anIndF; ind <= anIndL; ind++)
        {
            Value(ind, anArr1, anArr2);
            theOut.Write(string.Create(CultureInfo.InvariantCulture,
                                       $"{ind,4}  [{F(anArr1[1].X())} {F(anArr1[1].Y())} {F(anArr1[1].Z())}]  "
                                       + $"[{F(anArr2[1].X())} {F(anArr2[1].Y())}]  [{F(anArr2[2].X())} {F(anArr2[2].Y())}]\n"));
        }
    }

    // printf's "%+10.20f": a sign always, twenty decimals.
    private static string F(double theValue)
    {
        string aStr = theValue.ToString("F20", CultureInfo.InvariantCulture);
        return aStr.StartsWith('-') ? aStr : "+" + aStr;
    }
}
