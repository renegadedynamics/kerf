// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFcts;

namespace Kerf.Engine.Fillets;

// Kerf (unit 15AK3): the corners of the path of an evolving fillet's ball that are past the tolerance, built. Not a
// port: OCCT's sections aren't the envelope's (decision 2232) and it has no such corners.
internal partial class FilBuilder
{
    // The corners to build: the stripe's spine, the two contacts, the end of the surface before the corner and the
    // start of the one after it, and whether those two surfaces cross (a crease) or leave a gap (a piece of the ball).
    private readonly List<(Spine Spine, Point3D A, Point3D B, EnvelopeEnd From, EnvelopeEnd To, bool Crosses)> _envelopePieces = [];

    /// <summary>How many corners were built as a piece of the ball, and how many as a crease.</summary>
    internal (int Lunes, int Creases) EnvelopeCornersMade { get; private set; }

    /// <summary>
    /// Builds each corner of the stripe that <see cref="RefuseEnvelopeCorner"/> found past the tolerance, the two
    /// surfaces beside it having been left to end on their own circles. Where the circles leave a gap, the piece of
    /// the ball between them goes into the stripe as a surface of its own between the two walked ones
    /// (<see cref="EnvelopeLune"/>): its sides along the faces are the contacts, single points, as a sphere's are where
    /// ChFiKPart turns a fillet about a vertex, and the data structure takes it as it takes those. Where the two
    /// surfaces cross, they are given the curve they cross in to end on (<see cref="MakeEnvelopeCrease"/>). A corner
    /// that can't be built refuses the stripe as before, saying why (<see cref="EnvelopeCornerUnbuilt"/>); and so do
    /// two neighbouring surfaces of an evolving fillet that don't end on one line.
    /// </summary>
    internal override void CompleteSetOfSurf(Stripe stripe)
    {
        Spine spine = stripe.Spine();
        NCollection_HSequence<SurfData> seq = stripe.SetOfSurfData();
        if (spine is null || seq is null)
        {
            return;
        }

        foreach ((Spine of, Point3D a, Point3D b, EnvelopeEnd from, EnvelopeEnd to, bool crosses) in _envelopePieces)
        {
            if (!ReferenceEquals(of, spine))
            {
                continue;
            }

            double same = 1.0e-3 * from.Radius;
            int at = 0, count = seq.Length();
            for (int j = 1; j <= count && at == 0; j++)
            {
                if (j == count && (!spine.IsPeriodic || count == 1))
                {
                    break;
                }

                SurfData before = seq.Value(j), after = seq.Value(j == count ? 1 : j + 1);
                Point3D p1 = before.VertexLastOnS1().Point(), p2 = before.VertexLastOnS2().Point();
                bool here = (p1.DistanceTo(a) < same && p2.DistanceTo(b) < same) || (p1.DistanceTo(b) < same && p2.DistanceTo(a) < same);
                if (here && after.VertexFirstOnS1().Point().DistanceTo(p1) < same && after.VertexFirstOnS2().Point().DistanceTo(p2) < same)
                {
                    at = j;
                }
            }

            if (at == 0)
            {
                Unbuilt("no two neighbouring surfaces of the stripe end on its ball");
            }

            bool built = at != 0 && (crosses ? MakeEnvelopeCrease(seq, at, from, to) : MakeEnvelopeLune(seq, at, from, to));
            if (!built)
            {
                spine.SetErrorStatus(ErrorStatus.EnvelopeCorner);
                throw new FailureException(crosses
                    ? "PerformSetOfSurf : the crease where the radius changes at a corner of the ball's path can't be built"
                    : "PerformSetOfSurf : the piece of the ball where the radius changes at a corner of its path can't be built");
            }
        }

        // What is left has to meet: each surface's last line on the next one's first. Where the ball turns about a
        // vertex between two edges it rests on, with its radius changing, the two aren't lines of one ball at all (the
        // corpus's buildevol/K3: 0.09 apart, an edge's tolerance covering it, its faces 1.4e-3 off the union of the
        // balls beside it), and the surface the ball sweeps there isn't built: refused.
        if (spine is not FilSpine { } fil || fil.IsConstant() || !EnvelopeCornersRefused || !EnvelopeEndsMeet)
        {
            return;
        }

        DsDataStructure dStr = _ds.ChangeDS();
        for (int j = 1; j < seq.Length(); j++)
        {
            SurfData before = seq.Value(j), after = seq.Value(j + 1);
            if (before.CreaseAtLast is not null || before.Surf() == 0 || after.Surf() == 0)
            {
                continue;
            }

            double apart = EnvelopeApart(dStr, before, after);
            EnvelopeJoinWorst = Math.Max(EnvelopeJoinWorst, apart);
            if (apart > EnvelopeJoinLimit)
            {
                spine.SetErrorStatus(ErrorStatus.EnvelopeCorner);
                Unbuilt(FormattableString.Invariant($"two neighbouring surfaces end {apart:G3} apart, on different balls"));
                throw new FailureException("PerformSetOfSurf : two of the surfaces the ball sweeps don't meet where its radius changes");
            }
        }
    }

    /// <summary>
    /// How far apart two neighbouring surfaces of an evolving fillet may end before it is refused: the approximation's
    /// tolerance. Those of the corpus that are built meet within a ninth of it.
    /// </summary>
    private double EnvelopeJoinLimit => _tolapp3d;

    /// <summary>The most any surface's last line is from the next one's first, over the stripes of evolving radius.</summary>
    internal double EnvelopeJoinWorst { get; private set; }

    // How far the one surface's last line is from the other's first, as curves: 9 points of the one, each against the
    // nearest of 65 of the other and then the nearest point of the line between that one's neighbours.
    private static double EnvelopeApart(DsDataStructure dStr, SurfData before, SurfData after)
    {
        FaceInterference f1 = before.InterferenceOnS1(), f2 = before.InterferenceOnS2();
        FaceInterference g1 = after.InterferenceOnS1(), g2 = after.InterferenceOnS2();
        if (f1.PCurveOnSurf() is null || f2.PCurveOnSurf() is null || g1.PCurveOnSurf() is null || g2.PCurveOnSurf() is null
            || dStr.Surface(before.Surf()).Surface() is not { } last || dStr.Surface(after.Surf()).Surface() is not { } first)
        {
            return 0.0;
        }

        Point2D uv1 = f1.PCurveOnSurf().Value(f1.LastParameter()), uv2 = f2.PCurveOnSurf().Value(f2.LastParameter());
        Point2D next1 = g1.PCurveOnSurf().Value(g1.FirstParameter()), next2 = g2.PCurveOnSurf().Value(g2.FirstParameter());
        const int Count = 64;
        Point3D Line(double share) => first.Value(next1.X + share * (next2.X - next1.X), next1.Y + share * (next2.Y - next1.Y));
        Span<Point3D> line = stackalloc Point3D[Count + 1];
        for (int i = 0; i <= Count; i++)
        {
            line[i] = Line(i / (double)Count);
        }

        double worst = 0.0;
        for (int k = 0; k <= 8; k++)
        {
            double share = k / 8.0;
            Point3D p = last.Value(uv1.X + share * (uv2.X - uv1.X), uv1.Y + share * (uv2.Y - uv1.Y));
            int nearest = 0;
            for (int i = 1; i <= Count; i++)
            {
                if (line[i].SquareDistanceTo(p) < line[nearest].SquareDistanceTo(p))
                {
                    nearest = i;
                }
            }

            double lo = Math.Max(nearest - 1, 0) / (double)Count, hi = Math.Min(nearest + 1, Count) / (double)Count;
            for (int iter = 0; iter < 24; iter++)
            {
                double a = lo + (hi - lo) / 3.0, b = hi - (hi - lo) / 3.0;
                if (Line(a).SquareDistanceTo(p) < Line(b).SquareDistanceTo(p))
                {
                    hi = b;
                }
                else
                {
                    lo = a;
                }
            }

            worst = Math.Max(worst, Line(0.5 * (lo + hi)).DistanceTo(p));
        }

        return worst;
    }

    /// <summary>Why the last corner that wasn't built wasn't.</summary>
    internal string EnvelopeCornerUnbuilt { get; private set; } = "";

    private bool Unbuilt(string why)
    {
        EnvelopeCornerUnbuilt = why;
        return false;
    }

    private bool MakeEnvelopeLune(NCollection_HSequence<SurfData> seq, int at, in EnvelopeEnd from, in EnvelopeEnd to)
    {
        if (at == seq.Length())
        {
            return Unbuilt("the stripe's own ends meet there");
        }

        DsDataStructure dStr = _ds.ChangeDS();
        SurfData before = seq.Value(at), after = seq.Value(at + 1);
        FaceInterference f1 = before.InterferenceOnS1(), f2 = before.InterferenceOnS2();
        FaceInterference g1 = after.InterferenceOnS1(), g2 = after.InterferenceOnS2();
        if (f1.PCurveOnSurf() is null || f2.PCurveOnSurf() is null || g1.PCurveOnSurf() is null || g2.PCurveOnSurf() is null)
        {
            return Unbuilt("a surface beside it has no lines along the faces");
        }

        // The walked surfaces have their sections along V, or along U where the ball rests on an edge; the piece is
        // made the way the surface before it is, so that it takes that surface's orientation and transitions.
        Point2D uv1 = f1.PCurveOnSurf().Value(f1.LastParameter()), uv2 = f2.PCurveOnSurf().Value(f2.LastParameter());
        Point2D start = f1.PCurveOnSurf().Value(f1.FirstParameter());
        Point2D next1 = g1.PCurveOnSurf().Value(g1.FirstParameter()), next2 = g2.PCurveOnSurf().Value(g2.FirstParameter());
        Surface last = dStr.Surface(before.Surf()).Surface(), first = dStr.Surface(after.Surf()).Surface();
        bool acrossU = Math.Abs(uv1.Y - uv2.Y) < Precision.PConfusion();
        double side1 = acrossU ? uv1.X : uv1.Y, side2 = acrossU ? uv2.X : uv2.Y;
        if (Math.Abs(side1 - side2) < Precision.PConfusion() || (acrossU ? uv1.Y <= start.Y : uv1.X <= start.X))
        {
            return Unbuilt("the surface before it is not one walked along the stripe");
        }

        BSplineSurface lune = EnvelopeLune.Make(from, to, last.Value(uv1.X, uv1.Y), side1, side2, acrossU);
        if (lune is null)
        {
            return Unbuilt("its contacts are too near each other or opposite, or its arcs too wide");
        }

        // It has to begin on the one surface's last line and end on the other's first, and go the way they go. The
        // lines are compared as curves, each with the circle it should be: a surface's sections needn't be
        // parametrised as the piece's are (they aren't where the ball rests on an edge), and the data structure makes
        // each curve's pcurve on its second surface to the curve's own parameter.
        Point3D On(double along, double across) => acrossU ? lune.Value(across, along) : lune.Value(along, across);
        for (int i = 0; i <= 8; i++)
        {
            double share = i / 8.0, across = side1 + share * (side2 - side1);
            Point3D on = last.Value(uv1.X + share * (uv2.X - uv1.X), uv1.Y + share * (uv2.Y - uv1.Y));
            Point3D onNext = first.Value(next1.X + share * (next2.X - next1.X), next1.Y + share * (next2.Y - next1.Y));
            if (OffCircle(from.Circle, on) > _tolapp3d || OffCircle(from.Circle, On(0.0, across)) > _tolapp3d
                || OffCircle(to.Circle, onNext) > _tolapp3d || OffCircle(to.Circle, On(1.0, across)) > _tolapp3d)
            {
                return Unbuilt("the two surfaces don't end on the ball's two circles");
            }
        }

        last.D1(0.5 * (uv1.X + uv2.X), 0.5 * (uv1.Y + uv2.Y), out _, out Vector3D du, out Vector3D dv);
        if ((acrossU ? dv : du).Dot(new Vector3D(On(0.0, 0.5 * (side1 + side2)), On(1.0, 0.5 * (side1 + side2)))) <= 0.0)
        {
            return Unbuilt("the surface before it goes the other way");
        }

        SurfData piece = new();
        piece.Copy(before);
        piece.ResetSimul();
        piece.ChangeSurf(dStr.AddSurface(new DsSurface(lune, Precision.Confusion())));
        piece.ChangeVertexFirstOnS1() = before.VertexLastOnS1();
        piece.ChangeVertexFirstOnS2() = before.VertexLastOnS2();
        piece.FirstSpineParam(before.LastSpineParam());
        piece.TwistOnS1(false);
        piece.TwistOnS2(false);
        ref FaceInterference i1 = ref piece.ChangeInterferenceOnS1();
        i1.SetInterference(ChFiKPart_IndexCurveInDS(null, dStr), f1.Transition(), null, EnvelopeSide(side1, acrossU));
        i1.SetFirstParameter(0.0);
        i1.SetLastParameter(1.0);
        ref FaceInterference i2 = ref piece.ChangeInterferenceOnS2();
        i2.SetInterference(ChFiKPart_IndexCurveInDS(null, dStr), f2.Transition(), null, EnvelopeSide(side2, acrossU));
        i2.SetFirstParameter(0.0);
        i2.SetLastParameter(1.0);
        seq.InsertAfter(at, piece);
        foreach (NCollection_List<int> surfaces in _eviMap.Values)
        {
            if (surfaces.Contains(before.Surf()))
            {
                surfaces.Append(piece.Surf());
            }
        }

        (int lunes, int creases) = EnvelopeCornersMade;
        EnvelopeCornersMade = (lunes + 1, creases);
        return true;
    }

    // How far the point is from the circle.
    private static double OffCircle(in EnvelopeCircle circle, in Point3D p)
    {
        Vector3D to = new(circle.Centre, p);
        double above = to.Dot(circle.Plane);
        double within = (to - above * circle.Plane).Magnitude - circle.Radius;
        return Math.Sqrt(above * above + within * within);
    }

    private static LineCurve2D EnvelopeSide(double side, bool acrossU) =>
        acrossU ? new LineCurve2D(new Point2D(side, 0.0), Constants.DY2d()) : new LineCurve2D(new Point2D(0.0, side), Constants.DX2d());

    // A walked surface with its walk's parameter first and its section's second, whichever of U and V they are.
    private readonly struct EnvelopePatch(Surface surface, bool acrossU)
    {
        public Point3D Value(double along, double across) => acrossU ? surface.Value(across, along) : surface.Value(along, across);

        public Point2D Uv(double along, double across) => acrossU ? new Point2D(across, along) : new Point2D(along, across);

        public void D1(double along, double across, out Point3D p, out Vector3D dAlong, out Vector3D dAcross)
        {
            if (acrossU)
            {
                surface.D1(across, along, out p, out dAcross, out dAlong);
            }
            else
            {
                surface.D1(along, across, out p, out dAlong, out dAcross);
            }
        }

        // The point of the surface nearest p, from a guess, and how far p is from it along its normal.
        public bool Height(in Point3D p, ref double along, ref double across, out double height)
        {
            height = 0.0;
            for (int i = 0; i < 30; i++)
            {
                D1(along, across, out Point3D q, out Vector3D d1, out Vector3D d2);
                Vector3D to = new(q, p);
                double a11 = d1.Dot(d1), a12 = d1.Dot(d2), a22 = d2.Dot(d2), b1 = d1.Dot(to), b2 = d2.Dot(to);
                double det = a11 * a22 - a12 * a12;
                if (det <= 1.0e-12 * a11 * a22)
                {
                    return false;
                }

                double s1 = (b1 * a22 - b2 * a12) / det, s2 = (a11 * b2 - a12 * b1) / det;
                along += s1;
                across += s2;
                if (s1 * s1 * a11 + s2 * s2 * a22 < 1.0e-24)
                {
                    D1(along, across, out q, out d1, out d2);
                    Vector3D normal = d1.Crossed(d2);
                    height = new Vector3D(q, p).Dot(normal) / normal.Magnitude;
                    return true;
                }
            }

            return false;
        }
    }

    /// <summary>
    /// Where the two sides' surfaces cross, each ending on its own circle beyond the other's, the surface the balls
    /// sweep has a crease: the curve the two cross in, from one contact to the other, between the two circles. It is
    /// found on the surfaces as they were approximated: at each of 31 lines along the earlier surface, the point where
    /// that line goes through the later one (the two are a height apart that changes sign there; near the contacts,
    /// where they are less than a millionth apart all the way, the point is taken between its neighbours'). The curve
    /// through those points and its pcurves on the two surfaces replace the line the surfaces would have met on, and
    /// each surface's face ends on it. The two faces aren't tangent across it: the edge isn't coded as smooth.
    /// </summary>
    private bool MakeEnvelopeCrease(NCollection_HSequence<SurfData> seq, int at, in EnvelopeEnd from, in EnvelopeEnd to)
    {
        const int Count = 32;
        if (at == seq.Length())
        {
            return Unbuilt("the stripe's own ends meet there");
        }

        DsDataStructure dStr = _ds.ChangeDS();
        SurfData before = seq.Value(at), after = seq.Value(at + 1);
        FaceInterference f1 = before.InterferenceOnS1(), f2 = before.InterferenceOnS2();
        FaceInterference g1 = after.InterferenceOnS1(), g2 = after.InterferenceOnS2();
        if (f1.PCurveOnSurf() is null || f2.PCurveOnSurf() is null || g1.PCurveOnSurf() is null || g2.PCurveOnSurf() is null
            || before.CreaseAtLast is not null || after.CreaseAtFirst is not null)
        {
            return Unbuilt("a surface beside it has no lines along the faces, or has a crease there already");
        }

        Point2D uv1 = f1.PCurveOnSurf().Value(f1.LastParameter()), uv2 = f2.PCurveOnSurf().Value(f2.LastParameter());
        Point2D start = f1.PCurveOnSurf().Value(f1.FirstParameter());
        Point2D next1 = g1.PCurveOnSurf().Value(g1.FirstParameter()), next2 = g2.PCurveOnSurf().Value(g2.FirstParameter());
        Point2D end = g1.PCurveOnSurf().Value(g1.LastParameter());
        bool acrossU = Math.Abs(uv1.Y - uv2.Y) < Precision.PConfusion(), nextAcrossU = Math.Abs(next1.Y - next2.Y) < Precision.PConfusion();
        EnvelopePatch last = new(dStr.Surface(before.Surf()).Surface(), acrossU), first = new(dStr.Surface(after.Surf()).Surface(), nextAcrossU);
        double side1 = acrossU ? uv1.X : uv1.Y, side2 = acrossU ? uv2.X : uv2.Y, walkLast = acrossU ? uv1.Y : uv1.X;
        double nextSide1 = nextAcrossU ? next1.X : next1.Y, nextSide2 = nextAcrossU ? next2.X : next2.Y, walkFirst = nextAcrossU ? next1.Y : next1.X;
        double length = walkLast - (acrossU ? start.Y : start.X), nextLength = (nextAcrossU ? end.Y : end.X) - walkFirst;
        if (Math.Abs(side1 - side2) < Precision.PConfusion() || Math.Abs(nextSide1 - nextSide2) < Precision.PConfusion() || length <= 0.0 || nextLength <= 0.0)
        {
            return Unbuilt("a surface beside it is not one walked along the stripe");
        }

        double[] back = new double[Count + 1];
        bool[] found = new bool[Count + 1];
        int reliable = 0;
        for (int i = 1; i < Count; i++)
        {
            double share = i / (double)Count, across = side1 + share * (side2 - side1);
            last.D1(walkLast, across, out Point3D onEnd, out Vector3D speed, out _);
            double apart = onEnd.DistanceTo(first.Value(walkFirst, nextSide1 + share * (nextSide2 - nextSide1)));
            double reach = 1.3 * apart / speed.Magnitude;
            double w = walkFirst, s = nextSide1 + share * (nextSide2 - nextSide1);
            if (reach > 0.5 * length || !first.Height(onEnd, ref w, ref s, out double high))
            {
                return Unbuilt("a surface beside it is shorter than the corner is wide");
            }

            if (Math.Abs(high) < 1.0e-6)
            {
                continue;
            }

            // The height of the earlier surface's line over the later surface changes sign between the two circles
            double lo = 0.0, hi = reach, heightLo = high;
            double wHi = w, sHi = s;
            if (!first.Height(last.Value(walkLast - hi, across), ref wHi, ref sHi, out double heightHi) || heightLo * heightHi >= 0.0)
            {
                return Unbuilt("the two surfaces don't cross between their circles");
            }

            for (int k = 0; k < 60 && hi - lo > 1.0e-13 * length; k++)
            {
                double mid = heightHi != heightLo && k % 3 != 2 ? lo - heightLo * (hi - lo) / (heightHi - heightLo) : 0.5 * (lo + hi);
                if (!first.Height(last.Value(walkLast - mid, across), ref w, ref s, out double height))
                {
                    return Unbuilt("the height of a point over the later surface wasn't found");
                }

                if (Math.Abs(height) < 1.0e-11)
                {
                    lo = hi = mid;
                }
                else if (height * heightLo > 0.0)
                {
                    lo = mid;
                    heightLo = height;
                }
                else
                {
                    hi = mid;
                    heightHi = height;
                }
            }

            back[i] = 0.5 * (lo + hi);
            found[i] = true;
            reliable++;
        }

        if (reliable < Count / 2)
        {
            return Unbuilt("the two surfaces cross too flatly to say where");
        }

        // The lines whose crossing is too flat to find take theirs between their neighbours', the contacts being ends
        found[0] = found[Count] = true;
        for (int i = 1; i < Count; i++)
        {
            if (found[i])
            {
                continue;
            }

            int upper = i + 1;
            while (!found[upper])
            {
                upper++;
            }

            back[i] = back[i - 1] + (back[upper] - back[i - 1]) / (upper - i + 1);
        }

        // The crossing's pcurve on the earlier surface goes through those points; the curve itself and its pcurve on
        // the later surface go through four times as many points of that one, so that the three agree between them
        // to some 1e-8, and nothing after this has to make them agree
        const int Fine = 4 * Count;
        NCollection_HArray1<Point2D> onLast = new(1, Count + 1);
        NCollection_HArray1<double> parameters = new(1, Count + 1);
        for (int i = 0; i <= Count; i++)
        {
            double share = i / (double)Count;
            parameters.SetValue(i + 1, share);
            onLast.SetValue(i + 1, last.Uv(walkLast - back[i], side1 + share * (side2 - side1)));
        }

        Curve3D curve;
        Curve2D pcurveLast, pcurveFirst;
        Surface lastSurface = dStr.Surface(before.Surf()).Surface(), firstSurface = dStr.Surface(after.Surf()).Surface();
        try
        {
            global::Kerf.Engine.Geometry.Api.Interpolate2D in2dLast = new(onLast, parameters, false, 1.0e-14);
            in2dLast.Perform();
            if (!in2dLast.IsDone)
            {
                return Unbuilt("the curve through the crossing points wasn't made");
            }

            pcurveLast = in2dLast.Curve();
            NCollection_HArray1<Point3D> points = new(1, Fine + 1);
            NCollection_HArray1<Point2D> onFirst = new(1, Fine + 1);
            NCollection_HArray1<double> fine = new(1, Fine + 1);
            double w = walkFirst, s = nextSide1;
            for (int i = 0; i <= Fine; i++)
            {
                double share = i / (double)Fine;
                Point2D uv = i == 0 ? uv1 : i == Fine ? uv2 : pcurveLast.Value(share);
                Point3D p = lastSurface.Value(uv.X, uv.Y);
                if (i == 0 || i == Fine)
                {
                    w = walkFirst;
                    s = i == 0 ? nextSide1 : nextSide2;
                }
                else if (!first.Height(p, ref w, ref s, out _))
                {
                    w = walkFirst;
                    s = nextSide1 + share * (nextSide2 - nextSide1);
                    if (!first.Height(p, ref w, ref s, out _))
                    {
                        return Unbuilt("the height of a point over the later surface wasn't found");
                    }
                }

                fine.SetValue(i + 1, share);
                points.SetValue(i + 1, p);
                onFirst.SetValue(i + 1, first.Uv(w, s));
            }

            global::Kerf.Engine.Geometry.Api.ApiInterpolate in3d = new(points, fine, false, 1.0e-12);
            in3d.Perform();
            global::Kerf.Engine.Geometry.Api.Interpolate2D in2dFirst = new(onFirst, fine, false, 1.0e-14);
            in2dFirst.Perform();
            if (!in3d.IsDone || !in2dFirst.IsDone)
            {
                return Unbuilt("the curve through the crossing points wasn't made");
            }

            curve = in3d.Curve();
            pcurveFirst = in2dFirst.Curve();
        }
        catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
        {
            return Unbuilt("the curve through the crossing points wasn't made");
        }

        // How far apart the curve and its two pcurves are, between the points they share
        double apartMost = 0.0;
        for (int i = 0; i <= 2 * Fine; i++)
        {
            double t = i / (2.0 * Fine);
            Point3D p = curve.Value(t);
            Point2D a = pcurveLast.Value(t), b = pcurveFirst.Value(t);
            apartMost = Math.Max(apartMost, Math.Max(p.DistanceTo(lastSurface.Value(a.X, a.Y)), p.DistanceTo(firstSurface.Value(b.X, b.Y))));
        }

        if (apartMost > 0.5 * _tolapp3d)
        {
            return Unbuilt("the crossing's curve is past the tolerance from the surfaces");
        }

        EnvelopeCrease crease = new(curve, pcurveLast, pcurveFirst, 0.0, 1.0, Math.Max(2.0 * apartMost, Precision.Confusion()));
        before.CreaseAtLast = crease;
        after.CreaseAtFirst = crease;
        (int lunes, int creases) = EnvelopeCornersMade;
        EnvelopeCornersMade = (lunes, creases + 1);
        return true;
    }
}
