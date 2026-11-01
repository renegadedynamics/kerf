// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using static Occt.Contap_IType;
using static Occt.GeomAbs_CurveType;
using static Occt.IntSurf_TypeTrans;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// tools/hlrrec's main.cpp in C#, line for line: the same projectors, intervals, edge statuses, min-max boxes and
/// contours, the same calls in the same order, written in the same text (every double as its 64 bits in hex), so the
/// tests compare the port's text with native OCCT's recording.
/// </summary>
internal sealed class HlrRecorder
{
    private readonly StringBuilder gOut = new();

    public string Text => gOut.ToString();

    private void Hex(double theValue) =>
        gOut.Append(' ').Append(BitConverter.DoubleToUInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture));

    private void HexF(float theValue) =>
        gOut.Append(' ').Append(BitConverter.SingleToUInt32Bits(theValue).ToString("x8", CultureInfo.InvariantCulture));

    private void Tag(string theTag) => gOut.Append(theTag);

    private void Int(int theValue) => gOut.Append(' ').Append(theValue.ToString(CultureInfo.InvariantCulture));

    private void End() => gOut.Append('\n');

    private void Line(string theText) => gOut.Append(theText).Append('\n');

    private void Pnt(gp_Pnt P)
    {
        Hex(P.X());
        Hex(P.Y());
        Hex(P.Z());
    }

    private void Dir(gp_Dir D)
    {
        Hex(D.X());
        Hex(D.Y());
        Hex(D.Z());
    }

    private void Vec(gp_Vec V)
    {
        Hex(V.X());
        Hex(V.Y());
        Hex(V.Z());
    }

    private void Pnt2d(gp_Pnt2d P)
    {
        Hex(P.X());
        Hex(P.Y());
    }

    private void Vec2d(gp_Vec2d V)
    {
        Hex(V.X());
        Hex(V.Y());
    }

    private void Trsf(gp_Trsf T)
    {
        for (int r = 1; r <= 3; r++)
        {
            for (int c = 1; c <= 4; c++)
            {
                Hex(T.Value(r, c));
            }
        }
    }

    // ------------------------------------------------------------------------------------------------------------
    // HLRAlgo_Projector

    private static readonly double[,] THE_PTS =
        { { 0.0, 0.0, 0.0 }, { 1.0, 2.0, 3.0 }, { -4.5, 0.25, 7.0 }, { 10.0, -3.0, -2.0 }, { 0.1, 0.2, -0.3 } };

    private void RecProjector(string theName, HLRAlgo_Projector P)
    {
        Line($"case projector {theName}");
        Tag("persp");
        Int(P.Perspective() ? 1 : 0);
        if (P.Perspective())
        {
            Hex(P.Focus());
        }
        End();
        Tag("trsf");
        Trsf(P.Transformation());
        End();
        Tag("inv");
        Trsf(P.InvertedTransformation());
        End();
        Tag("full");
        Trsf(P.FullTransformation());
        End();
        P.Directions(out gp_Vec2d D1, out gp_Vec2d D2, out gp_Vec2d D3);
        Tag("dirs");
        Vec2d(D1);
        Vec2d(D2);
        Vec2d(D3);
        End();
        for (int i = 0; i < 5; i++)
        {
            gp_Pnt aP = new(THE_PTS[i, 0], THE_PTS[i, 1], THE_PTS[i, 2]);
            gp_Pnt2d aP2 = new(-7.0, -7.0);
            P.Project(aP, ref aP2);
            P.Project(aP, out double X, out double Y, out double Z);
            gp_Vec aD = new(THE_PTS[(i + 1) % 5, 0] + 0.5, THE_PTS[(i + 2) % 5, 1] - 0.5, 1.0);
            P.Project(aP, aD, out gp_Pnt2d aP3, out gp_Vec2d aD3);
            gp_Pnt aT = aP;
            P.Transform(ref aT);
            gp_Vec aTV = aD;
            P.Transform(ref aTV);
            Tag("proj");
            Pnt2d(aP2);
            Hex(X);
            Hex(Y);
            Hex(Z);
            Pnt2d(aP3);
            Vec2d(aD3);
            Pnt(aT);
            Vec(aTV);
            End();
            gp_Lin aL = P.Shoot(THE_PTS[i, 0], THE_PTS[i, 1]);
            Tag("shoot");
            Pnt(aL.Location());
            Dir(aL.Direction());
            End();
        }
    }

    public void RecProjectors()
    {
        RecProjector("identity", new HLRAlgo_Projector());
        RecProjector("ax2 top", new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0))));
        RecProjector("ax2 front", new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, -1, 0), new gp_Dir(1, 0, 0))));
        RecProjector("ax2 axo", new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(1, -1, 1.4142135623730951),
                                                                 new gp_Dir(1, 1, 0))));
        RecProjector("ax2 axo0", new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 1),
                                                                  new gp_Dir(1.4142135623730951, -1, 1))));
        RecProjector("ax2 general", new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(1, 2, 3), new gp_Dir(0.3, -0.7, 0.5),
                                                                     new gp_Dir(0.7, 0.3, 0.0))));
        RecProjector("ax2 down", new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, -1), new gp_Dir(1, 0, 0))));
        RecProjector("persp", new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0.3, -0.7, 0.5),
                                                               new gp_Dir(0.7, 0.3, 0.0)), 50.0));
        RecProjector("persp top", new HLRAlgo_Projector(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)),
                                                        20.0));

        gp_Trsf aT = new();
        aT.SetTransformation(new gp_Ax3(new gp_Pnt(1, -2, 0.5), new gp_Dir(1, 1, 1), new gp_Dir(1, -1, 0)));
        gp_Trsf aS = new();
        aS.SetScale(new gp_Pnt(0.5, 0.5, 0.5), 2.5);
        gp_Trsf aTS = aS * aT;
        RecProjector("trsf", new HLRAlgo_Projector(aTS, false, 0.0));
        RecProjector("trsf persp", new HLRAlgo_Projector(aTS, true, 30.0));
        HLRAlgo_Projector aScaled = new(aTS, false, 0.0);
        aScaled.Scaled(true);
        RecProjector("trsf scaled", aScaled);
        HLRAlgo_Projector aScaledP = new(aTS, true, 30.0);
        aScaledP.Scaled(true);
        RecProjector("trsf persp scaled", aScaledP);
        RecProjector("trsf dirs", new HLRAlgo_Projector(aT, false, 0.0, new gp_Vec2d(1, 2), new gp_Vec2d(-3, 0.5),
                                                        new gp_Vec2d(0.25, 0.75)));
        HLRAlgo_Projector aSet = new();
        aSet.Set(aT, true, 12.5);
        RecProjector("set", aSet);
    }

    // ------------------------------------------------------------------------------------------------------------
    // Intrv_Intervals, HLRAlgo_EdgeStatus, HLRAlgo_EdgeIterator

    private void Intervals(Intrv_Intervals I)
    {
        Tag("intervals");
        Int(I.NbIntervals());
        for (int i = 1; i <= I.NbIntervals(); i++)
        {
            I.Value(i).Bounds(out double S, out float TS, out double E, out float TE);
            Hex(S);
            HexF(TS);
            Hex(E);
            HexF(TE);
        }
        End();
    }

    public void RecIntervals()
    {
        Line("case intervals unite");
        Intrv_Intervals I = new(new Intrv_Interval(0.0, 1.0e-6f, 10.0, 1.0e-6f));
        Intervals(I);
        I.Unite(new Intrv_Interval(12.0, 1.0e-3f, 14.0, 1.0e-3f));
        Intervals(I);
        I.Unite(new Intrv_Interval(-5.0, 0.0f, -2.0, 0.0f));
        Intervals(I);
        I.Unite(new Intrv_Interval(9.9995, 1.0e-3f, 12.0005, 1.0e-3f));
        Intervals(I);
        I.Unite(new Intrv_Interval(-3.0, 1.0e-2f, 1.0, 1.0e-2f));
        Intervals(I);
        I.Unite(new Intrv_Interval(-10.0, 20.0));
        Intervals(I);

        Line("case intervals subtract");
        Intrv_Intervals J = new(new Intrv_Interval(0.0, 1.0e-6f, 10.0, 1.0e-6f));
        J.Subtract(new Intrv_Interval(2.0, 1.0e-4f, 3.0, 1.0e-4f));
        Intervals(J);
        J.Subtract(new Intrv_Interval(-1.0, 1.0e-4f, 0.5, 1.0e-4f));
        Intervals(J);
        J.Subtract(new Intrv_Interval(9.0, 1.0e-4f, 11.0, 1.0e-4f));
        Intervals(J);
        J.Subtract(new Intrv_Interval(3.0, 1.0e-4f, 3.5, 1.0e-4f));
        Intervals(J);
        J.Subtract(new Intrv_Interval(5.0, 1.0e-4f, 6.0, 1.0e-4f));
        Intervals(J);
        J.Subtract(new Intrv_Interval(-100.0, 100.0));
        Intervals(J);

        Line("case intervals intersect xunite");
        Intrv_Intervals K = new(new Intrv_Interval(0.0, 1.0e-6f, 4.0, 1.0e-6f));
        K.Unite(new Intrv_Interval(6.0, 1.0e-6f, 9.0, 1.0e-6f));
        Intrv_Intervals L = new(new Intrv_Interval(3.0, 1.0e-5f, 7.0, 1.0e-5f));
        L.Unite(new Intrv_Interval(8.5, 1.0e-5f, 12.0, 1.0e-5f));
        Intrv_Intervals KI = new(K);
        KI.Intersect(L);
        Intervals(KI);
        Intrv_Intervals KX = new(K);
        KX.XUnite(L);
        Intervals(KX);
        Intrv_Intervals KS = new(K);
        KS.Subtract(L);
        Intervals(KS);
        Intrv_Intervals KU = new(K);
        KU.Unite(L);
        Intervals(KU);
        KI.Intersect(new Intrv_Interval(3.5, 1.0e-5f, 8.75, 1.0e-5f));
        Intervals(KI);
        KX.XUnite(new Intrv_Interval(0.0, 1.0e-5f, 1.0, 1.0e-5f));
        Intervals(KX);

        Line("case interval positions");
        double[,] aB = { { -5.0, -3.0 }, { -5.0, 0.0 }, { -5.0, 2.0 }, { -5.0, 4.0 }, { 0.0, 2.0 }, { 1.0, 3.0 }, { 3.0, 6.0 } };
        Intrv_Interval aRef = new(0.0, 1.0e-3f, 4.0, 1.0e-3f);
        for (int i = 0; i < 7; i++)
        {
            Intrv_Interval aI = new(aB[i, 0], 1.0e-3f, aB[i, 1], 1.0e-3f);
            Tag("pos");
            Int((int)aI.Position(aRef));
            Int((int)aRef.Position(aI));
            Int(aI.IsBefore(aRef) ? 1 : 0);
            Int(aI.IsAfter(aRef) ? 1 : 0);
            Int(aI.IsInside(aRef) ? 1 : 0);
            Int(aI.IsEnclosing(aRef) ? 1 : 0);
            Int(aI.IsJustBefore(aRef) ? 1 : 0);
            Int(aI.IsJustAfter(aRef) ? 1 : 0);
            Int(aI.IsOverlappingAtStart(aRef) ? 1 : 0);
            Int(aI.IsOverlappingAtEnd(aRef) ? 1 : 0);
            Int(aI.IsJustEnclosingAtStart(aRef) ? 1 : 0);
            Int(aI.IsJustEnclosingAtEnd(aRef) ? 1 : 0);
            Int(aI.IsJustOverlappingAtStart(aRef) ? 1 : 0);
            Int(aI.IsJustOverlappingAtEnd(aRef) ? 1 : 0);
            Int(aI.IsSimilar(aRef) ? 1 : 0);
            Int(aI.IsProbablyEmpty() ? 1 : 0);
            End();
        }
        Intrv_Interval aDef = new();
        Intrv_Interval aTwo = new(1.5, -2.0);
        Tag("defaults");
        Hex(aDef.Start());
        HexF(aDef.TolStart());
        Hex(aDef.End());
        HexF(aDef.TolEnd());
        Hex(aTwo.Start());
        HexF(aTwo.TolStart());
        Hex(aTwo.End());
        HexF(aTwo.TolEnd());
        End();
    }

    private void Status(HLRAlgo_EdgeStatus S)
    {
        Tag("status");
        Int(S.AllHidden() ? 1 : 0);
        Int(S.AllVisible() ? 1 : 0);
        Int(S.NbVisiblePart());
        for (int i = 1; i <= S.NbVisiblePart(); i++)
        {
            S.VisiblePart(i, out double A, out float TA, out double B, out float TB);
            Hex(A);
            HexF(TA);
            Hex(B);
            HexF(TB);
        }
        End();
        HLRAlgo_EdgeIterator It = new();
        Tag("visible");
        for (It.InitVisible(S); It.MoreVisible(); It.NextVisible())
        {
            It.Visible(out double A, out float TA, out double B, out float TB);
            Hex(A);
            HexF(TA);
            Hex(B);
            HexF(TB);
        }
        End();
        Tag("hidden");
        for (It.InitHidden(S); It.MoreHidden(); It.NextHidden())
        {
            It.Hidden(out double A, out float TA, out double B, out float TB);
            Hex(A);
            HexF(TA);
            Hex(B);
            HexF(TB);
        }
        End();
    }

    public void RecStatus()
    {
        Line("case edgestatus hide");
        HLRAlgo_EdgeStatus S = new(0.0, 1.0e-5f, 1.0, 1.0e-5f);
        Status(S);
        S.Hide(0.2, 1.0e-5f, 0.3, 1.0e-5f, false, false);
        Status(S);
        S.Hide(0.5, 1.0e-5f, 0.6, 1.0e-5f, true, false); // on the face: nothing
        Status(S);
        S.Hide(0.5, 1.0e-5f, 0.6, 1.0e-5f, false, true);
        Status(S);
        S.Hide(-0.1, 1.0e-5f, 0.05, 1.0e-5f, false, false);
        Status(S);
        S.Hide(0.95, 1.0e-5f, 1.2, 1.0e-5f, false, false);
        Status(S);
        S.Hide(0.25, 1.0e-5f, 0.55, 1.0e-5f, false, false);
        Status(S);
        S.Hide(0.0, 1.0e-5f, 1.0, 1.0e-5f, false, false);
        Status(S);

        Line("case edgestatus all");
        HLRAlgo_EdgeStatus T = new(-2.0, 1.0e-4f, 3.0, 1.0e-4f);
        T.HideAll();
        Status(T);
        T.ShowAll();
        Status(T);
        T.Hide(-2.0, 1.0e-4f, -1.99995, 1.0e-4f, false, false);
        Status(T);
        T.Initialize(1.0, 1.0e-3f, 2.0, 1.0e-3f);
        Status(T);
        T.Hide(1.0005, 1.0e-3f, 1.9995, 1.0e-3f, false, false);
        Status(T);
        HLRAlgo_EdgeStatus U = new();
        // the default status: no visible part (its hidden iteration reads past the empty intervals in OCCT)
        Tag("default");
        Int(U.AllHidden() ? 1 : 0);
        Int(U.AllVisible() ? 1 : 0);
        Int(U.NbVisiblePart());
        End();
    }

    // ------------------------------------------------------------------------------------------------------------
    // HLRAlgo min-max

    public void RecMinMax()
    {
        Line("case minmax");
        double[] Min = new double[16], Max = new double[16];
        HLRAlgo.InitMinMax(1.0e100, Min, Max);
        for (int i = 0; i < 5; i++)
        {
            HLRAlgo.UpdateMinMax(THE_PTS[i, 0], THE_PTS[i, 1], THE_PTS[i, 2], Min, Max);
        }
        HLRAlgo.EnlargeMinMax(0.125, Min, Max);
        Tag("box");
        for (int i = 0; i < 16; i++)
        {
            Hex(Min[i]);
            Hex(Max[i]);
        }
        End();
        HLRAlgo_EdgesBlock.MinMaxIndices aMin = new(), aMax = new(), aMM = new(), aMin2 = new(), aMax2 = new();
        for (int i = 0; i < 8; i++)
        {
            aMin.Min[i] = (int)((Min[i] + 20.0) * 100.0);
            aMin.Max[i] = (int)((Min[i + 8] + 20.0) * 100.0);
            aMax.Min[i] = (int)((Max[i] + 20.0) * 100.0);
            aMax.Max[i] = (int)((Max[i + 8] + 20.0) * 100.0);
        }
        HLRAlgo.EncodeMinMax(ref aMin, ref aMax, ref aMM);
        HLRAlgo.DecodeMinMax(aMM, ref aMin2, ref aMax2);
        Tag("encoded");
        for (int i = 0; i < 8; i++)
        {
            Int(aMM.Min[i]);
            Int(aMM.Max[i]);
        }
        End();
        Tag("decoded");
        for (int i = 0; i < 8; i++)
        {
            Int(aMin2.Min[i]);
            Int(aMin2.Max[i]);
            Int(aMax2.Min[i]);
            Int(aMax2.Max[i]);
        }
        End();
        Tag("size");
        Hex(HLRAlgo.SizeBox(ref aMin2, ref aMax2));
        End();
        HLRAlgo_EdgesBlock.MinMaxIndices aOMin = aMin2, aOMax = aMax2;
        for (int i = 0; i < 8; i++)
        {
            aMin.Min[i] = aMin2.Min[i] - 3 + i;
            aMin.Max[i] = aMin2.Max[i] + 2 - i;
            aMax.Min[i] = aMax2.Min[i] + 5 - i;
            aMax.Max[i] = aMax2.Max[i] - 4 + i;
        }
        HLRAlgo.AddMinMax(ref aMin, ref aMax, ref aOMin, ref aOMax);
        Tag("added");
        for (int i = 0; i < 8; i++)
        {
            Int(aOMin.Min[i]);
            Int(aOMin.Max[i]);
            Int(aOMax.Min[i]);
            Int(aOMax.Max[i]);
        }
        End();
    }

    // ------------------------------------------------------------------------------------------------------------
    // Contap

    private void Transition(IntSurf_Transition T)
    {
        Int((int)T.TransitionType());
        if (T.TransitionType() != IntSurf_Undecided)
        {
            Int(T.IsTangent() ? 1 : 0);
        }
        if (T.TransitionType() == IntSurf_Touch)
        {
            Int((int)T.Situation());
            Int(T.IsOpposite() ? 1 : 0);
        }
    }

    private void ContAna(Contap_ContAna C)
    {
        Tag("contana");
        Int(C.IsDone() ? 1 : 0);
        if (C.IsDone())
        {
            Int(C.NbContours());
            if (C.NbContours() > 0)
            {
                Int((int)C.TypeContour());
                if (C.TypeContour() == GeomAbs_Circle)
                {
                    gp_Circ aC = C.Circle();
                    Pnt(aC.Location());
                    Dir(aC.Axis().Direction());
                    Dir(aC.XAxis().Direction());
                    Hex(aC.Radius());
                }
                else
                {
                    for (int i = 1; i <= C.NbContours(); i++)
                    {
                        gp_Lin aL = C.Line(i);
                        Pnt(aL.Location());
                        Dir(aL.Direction());
                    }
                }
            }
        }
        End();
    }

    public void RecContAna()
    {
        gp_Sphere aSph = new(new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Cylinder aCyl = new(new gp_Ax3(new gp_Pnt(-1, 0, 2), new gp_Dir(0.2, 0.1, 1), new gp_Dir(1, 0, -0.2)), 3.0);
        gp_Cone aCone = new(new gp_Ax3(new gp_Pnt(0, 1, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 0.4, 2.0);
        gp_Dir aD1 = new(0.3, -0.4, 0.8), aD2 = new(0, 0, 1), aD3 = new(1, 0, 0);
        gp_Pnt aEye = new(20, -15, 30), aEyeIn = new(1, 2, 4);
        Line("case contana sphere");
        Contap_ContAna C = new();
        C.Perform(aSph, aD1);
        ContAna(C);
        C.Perform(aSph, aD2);
        ContAna(C);
        C.Perform(aSph, aD3, 0.2);
        ContAna(C);
        C.Perform(aSph, aEye);
        ContAna(C);
        C.Perform(aSph, aEyeIn);
        ContAna(C);
        Line("case contana cylinder");
        C.Perform(aCyl, aD1);
        ContAna(C);
        C.Perform(aCyl, new gp_Dir(0.2, 0.1, 1));
        ContAna(C);
        C.Perform(aCyl, aD3, 0.3);
        ContAna(C);
        C.Perform(aCyl, aD3, -0.3);
        ContAna(C);
        C.Perform(aCyl, aEye);
        ContAna(C);
        C.Perform(aCyl, aEyeIn);
        ContAna(C);
        Line("case contana cone");
        C.Perform(aCone, aD1);
        ContAna(C);
        C.Perform(aCone, aD3);
        ContAna(C);
        C.Perform(aCone, aD2);
        ContAna(C);
        C.Perform(aCone, aD3, 0.1);
        ContAna(C);
        C.Perform(aCone, aD1, 0.05);
        ContAna(C);
        C.Perform(aCone, aEye);
        ContAna(C);
    }

    private void Contour(Contap_Contour C)
    {
        Tag("done");
        Int(C.IsDone() ? 1 : 0);
        End();
        if (!C.IsDone())
        {
            return;
        }
        Tag("lines");
        Int(C.NbLines());
        End();
        for (int l = 1; l <= C.NbLines(); l++)
        {
            Contap_Line L = C.Line(l);
            Tag("line");
            Int((int)L.TypeContour());
            Int((int)L.TransitionOnS());
            Int(L.NbVertex());
            switch (L.TypeContour())
            {
                case Contap_Lin:
                {
                    gp_Lin aL = L.Line();
                    Pnt(aL.Location());
                    Dir(aL.Direction());
                    break;
                }
                case Contap_Circle:
                {
                    gp_Circ aC = L.Circle();
                    Pnt(aC.Location());
                    Dir(aC.Axis().Direction());
                    Dir(aC.XAxis().Direction());
                    Hex(aC.Radius());
                    break;
                }
                case Contap_Walking:
                    Int(L.NbPnts());
                    break;
                case Contap_Restriction:
                    Hex(L.Arc().FirstParameter());
                    Hex(L.Arc().LastParameter());
                    break;
            }
            End();
            if (L.TypeContour() == Contap_Walking)
            {
                for (int i = 1; i <= L.NbPnts(); i++)
                {
                    L.Point(i).ParametersOnS2(out double U, out double V);
                    Tag("p");
                    Pnt(L.Point(i).Value());
                    Hex(U);
                    Hex(V);
                    End();
                }
            }
            for (int i = 1; i <= L.NbVertex(); i++)
            {
                Contap_Point P = L.Vertex(i);
                P.Parameters(out double U, out double V);
                Tag("v");
                Pnt(P.Value());
                Hex(U);
                Hex(V);
                Hex(P.ParameterOnLine());
                Int(P.IsVertex() ? 1 : 0);
                Int(P.IsMultiple() ? 1 : 0);
                Int(P.IsInternal() ? 1 : 0);
                Int(P.IsOnArc() ? 1 : 0);
                if (P.IsOnArc())
                {
                    Hex(P.ParameterOnArc());
                    Transition(P.TransitionOnLine());
                    Transition(P.TransitionOnArc());
                }
                End();
            }
        }
    }

    internal static TopoDS_Face FirstFace(TopoDS_Shape S, int theRank)
    {
        TopExp_Explorer anExp = new(S, TopAbs_FACE);
        for (int i = 1; i < theRank; i++)
        {
            anExp.Next();
        }
        return TopoDS.Face(anExp.Current());
    }

    internal static Geom_BSplineSurface Bump()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 5, 1, 4);
        for (int i = 1; i <= 5; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double x = (i - 1) * 2.5, y = (j - 1) * 3.0;
                double z = 4.0 * Math.Sin(0.6 * i) * Math.Cos(0.5 * j) + 0.3 * i * j;
                aPoles[i, j] = new gp_Pnt(x, y, z);
            }
        }
        NCollection_Array1<double> aUKnots = new(1, 3), aVKnots = new(1, 2);
        NCollection_Array1<int> aUMults = new(1, 3), aVMults = new(1, 2);
        aUKnots[1] = 0.0;
        aUKnots[2] = 0.4;
        aUKnots[3] = 1.0;
        aUMults[1] = 4;
        aUMults[2] = 1;
        aUMults[3] = 4;
        aVKnots[1] = 0.0;
        aVKnots[2] = 1.0;
        aVMults[1] = 4;
        aVMults[2] = 4;
        return new Geom_BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 3, 3);
    }

    private void RecFace(string theName, TopoDS_Face theFace)
    {
        TopoDS_Face aF = TopoDS.Face(theFace.Oriented(TopAbs_FORWARD));
        BRepTopAdaptor_Tool aTool = new(aF, Precision.PConfusion());
        BRepTopAdaptor_TopolTool aDomain = aTool.GetTopolTool();
        Adaptor3d_Surface aSurface = aTool.GetSurface()!;

        gp_Vec[] aDirs = [new gp_Vec(0.3, -0.4, 0.8), new gp_Vec(1, 0, 0), new gp_Vec(0.6, 0.7, -0.2)];
        gp_Pnt[] aEyes = [new gp_Pnt(40, -25, 30), new gp_Pnt(-3, 50, 12)];
        for (int i = 0; i < 3; i++)
        {
            Line($"case contour {theName} dir {i}");
            try
            {
                Contap_Contour C = new(aSurface, aDomain, aDirs[i]);
                Contour(C);
            }
            catch (Standard_Failure)
            {
                Line("exception");
            }
        }
        for (int i = 0; i < 2; i++)
        {
            Line($"case contour {theName} eye {i}");
            try
            {
                Contap_Contour C = new(aEyes[i]);
                C.Perform(aSurface, aDomain);
                Contour(C);
            }
            catch (Standard_Failure)
            {
                Line("exception");
            }
        }
        Line($"case contour {theName} draft");
        try
        {
            Contap_Contour C = new();
            C.Perform(aSurface, aDomain, aDirs[0], 0.15);
            Contour(C);
        }
        catch (Standard_Failure)
        {
            Line("exception");
        }
    }

    /// <summary>The faces of the contour cases, by name (in the recorder's order).</summary>
    internal static IEnumerable<(string Name, TopoDS_Face Face)> Faces()
    {
        yield return ("sphere", FirstFace(new BRepPrimAPI_MakeSphere(new gp_Pnt(1, 2, 3), 10.0).Shape(), 1));
        yield return ("halfsphere", FirstFace(new BRepPrimAPI_MakeSphere(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)),
                                                                          8.0, 0.0, M_PI / 2.0, M_PI).Shape(), 1));
        yield return ("cylinder", FirstFace(new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)),
                                                                          5.0, 20.0).Shape(), 1));
        yield return ("cylinder part", FirstFace(new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(1, 1, 0), new gp_Dir(0.1, 0.2, 1)),
                                                                               4.0, 12.0, 4.0).Shape(), 1));
        yield return ("cone", FirstFace(new BRepPrimAPI_MakeCone(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)),
                                                                  10.0, 3.0, 15.0).Shape(), 1));
        yield return ("cone apex", FirstFace(new BRepPrimAPI_MakeCone(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)),
                                                                       6.0, 0.0, 9.0).Shape(), 1));
        yield return ("torus", FirstFace(new BRepPrimAPI_MakeTorus(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)),
                                                                    10.0, 3.0).Shape(), 1));
        yield return ("torus part", FirstFace(new BRepPrimAPI_MakeTorus(new gp_Ax2(new gp_Pnt(2, 0, 1), new gp_Dir(0, 1, 1)),
                                                                         8.0, 2.5, 0.0, M_PI, 4.0).Shape(), 1));
        yield return ("box", FirstFace(new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape(), 3));
        yield return ("bspline", new BRepBuilderAPI_MakeFace(Bump(), Precision.Confusion()).Face());
    }

    public void RecContours()
    {
        foreach ((string aName, TopoDS_Face aFace) in Faces())
        {
            RecFace(aName, aFace);
        }
    }

    /// <summary>The contours of one face only (the tests run the faces as separate cases).</summary>
    public void RecContours(string theName)
    {
        foreach ((string aName, TopoDS_Face aFace) in Faces())
        {
            if (aName == theName)
            {
                RecFace(aName, aFace);
            }
        }
    }
}
