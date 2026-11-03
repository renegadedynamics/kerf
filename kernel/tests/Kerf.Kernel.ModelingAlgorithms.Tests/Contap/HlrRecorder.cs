// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.HiddenLines.Contours;
using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Text;
using static Kerf.Engine.HiddenLines.Contours.ContourLineType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

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

    private void Pnt(Point3D P)
    {
        Hex(P.X());
        Hex(P.Y());
        Hex(P.Z());
    }

    private void Dir(Direction3D D)
    {
        Hex(D.X());
        Hex(D.Y());
        Hex(D.Z());
    }

    private void Vec(Vector3D V)
    {
        Hex(V.X());
        Hex(V.Y());
        Hex(V.Z());
    }

    private void Pnt2d(Point2D P)
    {
        Hex(P.X());
        Hex(P.Y());
    }

    private void Vec2d(Vector2D V)
    {
        Hex(V.X());
        Hex(V.Y());
    }

    private void Trsf(Transform3D T)
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

    private void RecProjector(string theName, AlgorithmProjector P)
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
        P.Directions(out Vector2D D1, out Vector2D D2, out Vector2D D3);
        Tag("dirs");
        Vec2d(D1);
        Vec2d(D2);
        Vec2d(D3);
        End();
        for (int i = 0; i < 5; i++)
        {
            Point3D aP = new(THE_PTS[i, 0], THE_PTS[i, 1], THE_PTS[i, 2]);
            Point2D aP2 = new(-7.0, -7.0);
            P.Project(aP, ref aP2);
            P.Project(aP, out double X, out double Y, out double Z);
            Vector3D aD = new(THE_PTS[(i + 1) % 5, 0] + 0.5, THE_PTS[(i + 2) % 5, 1] - 0.5, 1.0);
            P.Project(aP, aD, out Point2D aP3, out Vector2D aD3);
            Point3D aT = aP;
            P.Transform(ref aT);
            Vector3D aTV = aD;
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
            Line3D aL = P.Shoot(THE_PTS[i, 0], THE_PTS[i, 1]);
            Tag("shoot");
            Pnt(aL.Location());
            Dir(aL.Direction());
            End();
        }
    }

    public void RecProjectors()
    {
        RecProjector("identity", new AlgorithmProjector());
        RecProjector("ax2 top", new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))));
        RecProjector("ax2 front", new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, -1, 0), new Direction3D(1, 0, 0))));
        RecProjector("ax2 axo", new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, -1, 1.4142135623730951),
                                                                 new Direction3D(1, 1, 0))));
        RecProjector("ax2 axo0", new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 1),
                                                                  new Direction3D(1.4142135623730951, -1, 1))));
        RecProjector("ax2 general", new AlgorithmProjector(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0.3, -0.7, 0.5),
                                                                     new Direction3D(0.7, 0.3, 0.0))));
        RecProjector("ax2 down", new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, -1), new Direction3D(1, 0, 0))));
        RecProjector("persp", new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0.3, -0.7, 0.5),
                                                               new Direction3D(0.7, 0.3, 0.0)), 50.0));
        RecProjector("persp top", new AlgorithmProjector(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)),
                                                        20.0));

        Transform3D aT = new();
        aT.SetTransformation(new CoordinateSystem(new Point3D(1, -2, 0.5), new Direction3D(1, 1, 1), new Direction3D(1, -1, 0)));
        Transform3D aS = new();
        aS.SetScale(new Point3D(0.5, 0.5, 0.5), 2.5);
        Transform3D aTS = aS * aT;
        RecProjector("trsf", new AlgorithmProjector(aTS, false, 0.0));
        RecProjector("trsf persp", new AlgorithmProjector(aTS, true, 30.0));
        AlgorithmProjector aScaled = new(aTS, false, 0.0);
        aScaled.Scaled(true);
        RecProjector("trsf scaled", aScaled);
        AlgorithmProjector aScaledP = new(aTS, true, 30.0);
        aScaledP.Scaled(true);
        RecProjector("trsf persp scaled", aScaledP);
        RecProjector("trsf dirs", new AlgorithmProjector(aT, false, 0.0, new Vector2D(1, 2), new Vector2D(-3, 0.5),
                                                        new Vector2D(0.25, 0.75)));
        AlgorithmProjector aSet = new();
        aSet.Set(aT, true, 12.5);
        RecProjector("set", aSet);
    }

    // ------------------------------------------------------------------------------------------------------------
    // Intrv_Intervals, HLRAlgo_EdgeStatus, HLRAlgo_EdgeIterator

    private void Intervals(Intervals I)
    {
        Tag("intervals");
        Int(I.IntervalCount);
        for (int i = 1; i <= I.IntervalCount; i++)
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
        Intervals I = new(new IntersectionInterval(0.0, 1.0e-6f, 10.0, 1.0e-6f));
        Intervals(I);
        I.Unite(new IntersectionInterval(12.0, 1.0e-3f, 14.0, 1.0e-3f));
        Intervals(I);
        I.Unite(new IntersectionInterval(-5.0, 0.0f, -2.0, 0.0f));
        Intervals(I);
        I.Unite(new IntersectionInterval(9.9995, 1.0e-3f, 12.0005, 1.0e-3f));
        Intervals(I);
        I.Unite(new IntersectionInterval(-3.0, 1.0e-2f, 1.0, 1.0e-2f));
        Intervals(I);
        I.Unite(new IntersectionInterval(-10.0, 20.0));
        Intervals(I);

        Line("case intervals subtract");
        Intervals J = new(new IntersectionInterval(0.0, 1.0e-6f, 10.0, 1.0e-6f));
        J.Subtract(new IntersectionInterval(2.0, 1.0e-4f, 3.0, 1.0e-4f));
        Intervals(J);
        J.Subtract(new IntersectionInterval(-1.0, 1.0e-4f, 0.5, 1.0e-4f));
        Intervals(J);
        J.Subtract(new IntersectionInterval(9.0, 1.0e-4f, 11.0, 1.0e-4f));
        Intervals(J);
        J.Subtract(new IntersectionInterval(3.0, 1.0e-4f, 3.5, 1.0e-4f));
        Intervals(J);
        J.Subtract(new IntersectionInterval(5.0, 1.0e-4f, 6.0, 1.0e-4f));
        Intervals(J);
        J.Subtract(new IntersectionInterval(-100.0, 100.0));
        Intervals(J);

        Line("case intervals intersect xunite");
        Intervals K = new(new IntersectionInterval(0.0, 1.0e-6f, 4.0, 1.0e-6f));
        K.Unite(new IntersectionInterval(6.0, 1.0e-6f, 9.0, 1.0e-6f));
        Intervals L = new(new IntersectionInterval(3.0, 1.0e-5f, 7.0, 1.0e-5f));
        L.Unite(new IntersectionInterval(8.5, 1.0e-5f, 12.0, 1.0e-5f));
        Intervals KI = new(K);
        KI.Intersect(L);
        Intervals(KI);
        Intervals KX = new(K);
        KX.XUnite(L);
        Intervals(KX);
        Intervals KS = new(K);
        KS.Subtract(L);
        Intervals(KS);
        Intervals KU = new(K);
        KU.Unite(L);
        Intervals(KU);
        KI.Intersect(new IntersectionInterval(3.5, 1.0e-5f, 8.75, 1.0e-5f));
        Intervals(KI);
        KX.XUnite(new IntersectionInterval(0.0, 1.0e-5f, 1.0, 1.0e-5f));
        Intervals(KX);

        Line("case interval positions");
        double[,] aB = { { -5.0, -3.0 }, { -5.0, 0.0 }, { -5.0, 2.0 }, { -5.0, 4.0 }, { 0.0, 2.0 }, { 1.0, 3.0 }, { 3.0, 6.0 } };
        IntersectionInterval aRef = new(0.0, 1.0e-3f, 4.0, 1.0e-3f);
        for (int i = 0; i < 7; i++)
        {
            IntersectionInterval aI = new(aB[i, 0], 1.0e-3f, aB[i, 1], 1.0e-3f);
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
            Int(aI.IsProbablyEmpty ? 1 : 0);
            End();
        }
        IntersectionInterval aDef = new();
        IntersectionInterval aTwo = new(1.5, -2.0);
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

    private void Status(EdgeStatus S)
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
        EdgeIterator It = new();
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
        EdgeStatus S = new(0.0, 1.0e-5f, 1.0, 1.0e-5f);
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
        EdgeStatus T = new(-2.0, 1.0e-4f, 3.0, 1.0e-4f);
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
        EdgeStatus U = new();
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
        HlrAlgo.InitMinMax(1.0e100, Min, Max);
        for (int i = 0; i < 5; i++)
        {
            HlrAlgo.UpdateMinMax(THE_PTS[i, 0], THE_PTS[i, 1], THE_PTS[i, 2], Min, Max);
        }
        HlrAlgo.EnlargeMinMax(0.125, Min, Max);
        Tag("box");
        for (int i = 0; i < 16; i++)
        {
            Hex(Min[i]);
            Hex(Max[i]);
        }
        End();
        EdgesBlock.MinMaxIndices aMin = new(), aMax = new(), aMM = new(), aMin2 = new(), aMax2 = new();
        for (int i = 0; i < 8; i++)
        {
            aMin.Min[i] = (int)((Min[i] + 20.0) * 100.0);
            aMin.Max[i] = (int)((Min[i + 8] + 20.0) * 100.0);
            aMax.Min[i] = (int)((Max[i] + 20.0) * 100.0);
            aMax.Max[i] = (int)((Max[i + 8] + 20.0) * 100.0);
        }
        HlrAlgo.EncodeMinMax(ref aMin, ref aMax, ref aMM);
        HlrAlgo.DecodeMinMax(aMM, ref aMin2, ref aMax2);
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
        Hex(HlrAlgo.SizeBox(ref aMin2, ref aMax2));
        End();
        EdgesBlock.MinMaxIndices aOMin = aMin2, aOMax = aMax2;
        for (int i = 0; i < 8; i++)
        {
            aMin.Min[i] = aMin2.Min[i] - 3 + i;
            aMin.Max[i] = aMin2.Max[i] + 2 - i;
            aMax.Min[i] = aMax2.Min[i] + 5 - i;
            aMax.Max[i] = aMax2.Max[i] - 4 + i;
        }
        HlrAlgo.AddMinMax(ref aMin, ref aMax, ref aOMin, ref aOMax);
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

    private void Transition(WalkingTransition T)
    {
        Int((int)T.TransitionType());
        if (T.TransitionType() != WalkingTypeTrans.Undecided)
        {
            Int(T.IsTangent ? 1 : 0);
        }
        if (T.TransitionType() == WalkingTypeTrans.Touch)
        {
            Int((int)T.Situation());
            Int(T.IsOpposite ? 1 : 0);
        }
    }

    private void ContAna(ContAna C)
    {
        Tag("contana");
        Int(C.IsDone ? 1 : 0);
        if (C.IsDone)
        {
            Int(C.ContourCount);
            if (C.ContourCount > 0)
            {
                Int((int)C.TypeContour());
                if (C.TypeContour() == global::Kerf.Geometry.CurveKind.Circle)
                {
                    Circle3D aC = C.Circle();
                    Pnt(aC.Location());
                    Dir(aC.Axis().Direction());
                    Dir(aC.XAxis().Direction());
                    Hex(aC.Radius());
                }
                else
                {
                    for (int i = 1; i <= C.ContourCount; i++)
                    {
                        Line3D aL = C.Line(i);
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
        Sphere aSph = new(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Cylinder aCyl = new(new CoordinateSystem(new Point3D(-1, 0, 2), new Direction3D(0.2, 0.1, 1), new Direction3D(1, 0, -0.2)), 3.0);
        Cone aCone = new(new CoordinateSystem(new Point3D(0, 1, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 0.4, 2.0);
        Direction3D aD1 = new(0.3, -0.4, 0.8), aD2 = new(0, 0, 1), aD3 = new(1, 0, 0);
        Point3D aEye = new(20, -15, 30), aEyeIn = new(1, 2, 4);
        Line("case contana sphere");
        ContAna C = new();
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
        C.Perform(aCyl, new Direction3D(0.2, 0.1, 1));
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

    private void Contour(Contour C)
    {
        Tag("done");
        Int(C.IsDone ? 1 : 0);
        End();
        if (!C.IsDone)
        {
            return;
        }
        Tag("lines");
        Int(C.LineCount);
        End();
        for (int l = 1; l <= C.LineCount; l++)
        {
            ContourLine L = C.Line(l);
            Tag("line");
            Int((int)L.TypeContour());
            Int((int)L.TransitionOnS());
            Int(L.VertexCount);
            switch (L.TypeContour())
            {
                case ContourLineType.Lin:
                {
                    Line3D aL = L.Line();
                    Pnt(aL.Location());
                    Dir(aL.Direction());
                    break;
                }
                case ContourLineType.Circle:
                {
                    Circle3D aC = L.Circle();
                    Pnt(aC.Location());
                    Dir(aC.Axis().Direction());
                    Dir(aC.XAxis().Direction());
                    Hex(aC.Radius());
                    break;
                }
                case ContourLineType.Walking:
                    Int(L.PntCount);
                    break;
                case ContourLineType.Restriction:
                    Hex(L.Arc().FirstParameter());
                    Hex(L.Arc().LastParameter());
                    break;
            }
            End();
            if (L.TypeContour() == ContourLineType.Walking)
            {
                for (int i = 1; i <= L.PntCount; i++)
                {
                    L.Point(i).ParametersOnS2(out double U, out double V);
                    Tag("p");
                    Pnt(L.Point(i).Value());
                    Hex(U);
                    Hex(V);
                    End();
                }
            }
            for (int i = 1; i <= L.VertexCount; i++)
            {
                ContourPoint P = L.Vertex(i);
                P.Parameters(out double U, out double V);
                Tag("v");
                Pnt(P.Value());
                Hex(U);
                Hex(V);
                Hex(P.ParameterOnLine());
                Int(P.IsVertex ? 1 : 0);
                Int(P.IsMultiple ? 1 : 0);
                Int(P.IsInternal ? 1 : 0);
                Int(P.IsOnArc ? 1 : 0);
                if (P.IsOnArc)
                {
                    Hex(P.ParameterOnArc());
                    Transition(P.TransitionOnLine());
                    Transition(P.TransitionOnArc());
                }
                End();
            }
        }
    }

    internal static Face FirstFace(Shape S, int theRank)
    {
        Explorer anExp = new(S, ShapeType.Face);
        for (int i = 1; i < theRank; i++)
        {
            anExp.Next();
        }
        return ShapeCasts.Face(anExp.Current());
    }

    internal static BSplineSurface Bump()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 5, 1, 4);
        for (int i = 1; i <= 5; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double x = (i - 1) * 2.5, y = (j - 1) * 3.0;
                double z = 4.0 * Math.Sin(0.6 * i) * Math.Cos(0.5 * j) + 0.3 * i * j;
                aPoles[i, j] = new Point3D(x, y, z);
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
        return new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 3, 3);
    }

    private void RecFace(string theName, Face theFace)
    {
        Face aF = ShapeCasts.Face(theFace.Oriented(Forward));
        AdaptorTool aTool = new(aF, global::Kerf.Engine.Foundation.Precision.PConfusion());
        BrepTopolTool aDomain = aTool.GetTopolTool();
        SurfaceAdaptorBase aSurface = aTool.GetSurface()!;

        Vector3D[] aDirs = [new Vector3D(0.3, -0.4, 0.8), new Vector3D(1, 0, 0), new Vector3D(0.6, 0.7, -0.2)];
        Point3D[] aEyes = [new Point3D(40, -25, 30), new Point3D(-3, 50, 12)];
        for (int i = 0; i < 3; i++)
        {
            Line($"case contour {theName} dir {i}");
            try
            {
                Contour C = new(aSurface, aDomain, aDirs[i]);
                Contour(C);
            }
            catch (global::Kerf.Engine.Foundation.FailureException)
            {
                Line("exception");
            }
        }
        for (int i = 0; i < 2; i++)
        {
            Line($"case contour {theName} eye {i}");
            try
            {
                Contour C = new(aEyes[i]);
                C.Perform(aSurface, aDomain);
                Contour(C);
            }
            catch (global::Kerf.Engine.Foundation.FailureException)
            {
                Line("exception");
            }
        }
        Line($"case contour {theName} draft");
        try
        {
            Contour C = new();
            C.Perform(aSurface, aDomain, aDirs[0], 0.15);
            Contour(C);
        }
        catch (global::Kerf.Engine.Foundation.FailureException)
        {
            Line("exception");
        }
    }

    /// <summary>The faces of the contour cases, by name (in the recorder's order).</summary>
    internal static IEnumerable<(string Name, Face Face)> Faces()
    {
        yield return ("sphere", FirstFace(new MakeSphere(new Point3D(1, 2, 3), 10.0).Shape(), 1));
        yield return ("halfsphere", FirstFace(new MakeSphere(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                                                          8.0, 0.0, M_PI / 2.0, M_PI).Shape(), 1));
        yield return ("cylinder", FirstFace(new ApiMakeCylinder(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                                                          5.0, 20.0).Shape(), 1));
        yield return ("cylinder part", FirstFace(new ApiMakeCylinder(new Frame3D(new Point3D(1, 1, 0), new Direction3D(0.1, 0.2, 1)),
                                                                               4.0, 12.0, 4.0).Shape(), 1));
        yield return ("cone", FirstFace(new ApiMakeCone(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                                                  10.0, 3.0, 15.0).Shape(), 1));
        yield return ("cone apex", FirstFace(new ApiMakeCone(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)),
                                                                       6.0, 0.0, 9.0).Shape(), 1));
        yield return ("torus", FirstFace(new MakeTorus(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                                                    10.0, 3.0).Shape(), 1));
        yield return ("torus part", FirstFace(new MakeTorus(new Frame3D(new Point3D(2, 0, 1), new Direction3D(0, 1, 1)),
                                                                         8.0, 2.5, 0.0, M_PI, 4.0).Shape(), 1));
        yield return ("box", FirstFace(new MakeBox(10.0, 20.0, 30.0).Shape(), 3));
        yield return ("bspline", new MakeFace(Bump(), global::Kerf.Engine.Foundation.Precision.Confusion()).Face());
    }

    public void RecContours()
    {
        foreach ((string aName, Face aFace) in Faces())
        {
            RecFace(aName, aFace);
        }
    }

    /// <summary>The contours of one face only (the tests run the faces as separate cases).</summary>
    public void RecContours(string theName)
    {
        foreach ((string aName, Face aFace) in Faces())
        {
            if (aName == theName)
            {
                RecFace(aName, aFace);
            }
        }
    }
}
