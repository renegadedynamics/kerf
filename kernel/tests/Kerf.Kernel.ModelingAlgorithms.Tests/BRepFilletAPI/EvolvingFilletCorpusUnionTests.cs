// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// Unit 15AK2's probe (decisions 2337, 2342): the corpus's evolving fillets whose ball comes to rest on an edge, each
/// against the union of its balls, the balls read from the input shape and the fillet's contacts alone. It takes minutes, so
/// it runs only where <c>KERF_15AK2_UNION</c> names a directory, and writes what it reads there (<c>union.tsv</c>).
/// <para>
/// A ball is read where a new face's boundary lies on a face G of the input, tangent to it: centred on G's normal at that
/// point, its radius the centre's distance from the one other face of the input it comes to rest on, as a bounded surface
/// (so its edges with it), which is one root, a distance growing no faster than the radius. Which other face is told by the
/// new face's far boundary, which has to be on the ball. Where a new face's boundaries both lie on edges (the ball between
/// two edges) the ball is the one through the two points whose chords to them are square to the edges. 1,000 to a boundary,
/// carried to 30,001 by cubics. A point of a new face is then on the swept surface if it is on the boundary of the union.
/// </para>
/// </summary>
public class EvolvingFilletCorpusUnionTests(ITestOutputHelper output)
{
    private const double Approximation = 1e-4;
    private const int Exact = 1000, Dense = 30000, Coarse = 50;

    // buildevol's cases on restored shapes, less K2, which fails for another reason (decision 2217)
    public static TheoryData<string> Cases() =>
    [
        "H9", "I1", "I2", "I3", "I4", "I5", "I6", "I7", "I8", "I9", "J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9", "K1", "K3", "K4",
        "K5", "K6", "K7", "K8",
    ];

    private static string Metadata(string key) =>
        typeof(EvolvingFilletCorpusUnionTests).Assembly.GetCustomAttributes(typeof(System.Reflection.AssemblyMetadataAttribute), false)
            .Cast<System.Reflection.AssemblyMetadataAttribute>().Single(a => a.Key == key).Value!;

    // The script's shape, its laws and the area it holds the result to: restore [locate_data_file X] s, updatevol s_N u r u r
    // ..., checkprops result -s A
    private static (Shape Shape, List<(int Edge, double[] Law)> Laws, bool Sorted, double Area)? Script(string name)
    {
        string path = Path.Combine(Metadata("OcctSourceDir"), "tests", "blend", "buildevol", name);
        if (!File.Exists(path))
        {
            return null;
        }

        Shape? shape = null;
        bool sorted = false;
        double area = double.NaN;
        List<(int, double[])> laws = [];
        foreach (string raw in File.ReadAllLines(path))
        {
            string line = raw.Trim();
            if (line.StartsWith("restore", StringComparison.Ordinal))
            {
                string file = line.Split('[', ']')[1].Split(' ', StringSplitOptions.RemoveEmptyEntries)[1];
                string data = Path.Combine(Metadata("OcctTestDataDir"), "brep", file);
                if (!File.Exists(data) || !BrepTools.Read(out Shape read, data))
                {
                    return null;
                }

                shape = read;
            }
            else if (line.StartsWith("nexplode", StringComparison.Ordinal))
            {
                sorted = true;
            }
            else if (line.StartsWith("checkprops result -s ", StringComparison.Ordinal))
            {
                area = double.Parse(line.Split(' ', StringSplitOptions.RemoveEmptyEntries)[3], CultureInfo.InvariantCulture);
            }
            else if (line.StartsWith("updatevol", StringComparison.Ordinal))
            {
                string[] words = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                laws.Add((int.Parse(words[1][2..], CultureInfo.InvariantCulture) - 1,
                          [.. words.Skip(2).Select(w => double.Parse(w.Replace("*SCALE2", "", StringComparison.Ordinal), CultureInfo.InvariantCulture))]));
            }
        }

        return shape is { } s ? (s, laws, sorted, area) : null;
    }

    private static MakeFillet Fillet(Shape shape, List<(int Edge, double[] Law)> laws, bool sorted, bool refused, bool meet)
    {
        var chfi3d = new MakeFillet(shape);
        chfi3d.InternalBuilder().EnvelopeCornersRefused = refused;
        chfi3d.InternalBuilder().EnvelopeEndsMeet = meet;
        Edge[] edges = [.. shape.AllEdges()];
        if (sorted)
        {
            // nexplode's order: by the centre of mass, x times 999 and y times 99 and z times 0.9
            double Key(Edge e)
            {
                Kerf.Engine.Measurement.Properties properties = new();
                Kerf.Engine.Measurement.MassPropertiesBuilder.LinearProperties(e, properties);
                Point3D g = properties.CentreOfMass();
                return g.X * 999 + g.Y * 99 + g.Z * 0.9;
            }

            edges = [.. edges.Select(e => (e, Key(e))).OrderBy(x => x.Item2).Select(x => x.e)];
        }

        foreach ((int edge, double[] law) in laws)
        {
            NCollection_Array1<Point2D> points = new(1, law.Length / 2);
            for (int i = 0; i < law.Length / 2; i++)
            {
                points.SetValue(i + 1, new Point2D(law[2 * i], law[2 * i + 1]));
            }

            chfi3d.Add(points, edges[edge]);
        }

        try
        {
            chfi3d.Build();
        }
        catch (Exception)
        {
            // not done
        }

        return chfi3d;
    }

    [Theory]
    [MemberData(nameof(Cases))]
    public void The_corpuss_fillets_onto_edges_against_the_union_of_their_balls(string name)
    {
        if (Environment.GetEnvironmentVariable("KERF_15AK2_UNION") is not { Length: > 0 } directory || Script(name) is not { } script
            || (Environment.GetEnvironmentVariable("KERF_15AK2_CASES") is { Length: > 0 } only && !only.Split(',').Contains(name)))
        {
            return;
        }

        System.Text.StringBuilder text = new();
        // As a user gets it, its corners past the tolerance built (unit 15AK3); where that is refused, with the refusal off
        MakeFillet asBuilt = Fillet(script.Shape, script.Laws, script.Sorted, refused: true, meet: true);
        bool built = asBuilt.IsDone;
        MakeFillet fillet = built ? asBuilt : Fillet(script.Shape, script.Laws, script.Sorted, refused: false, meet: true);
        double Tolerance(MakeFillet f) => f.IsDone ? f.Shape().AllEdges().Max(e => Tool.Tolerance(e)) : double.NaN;
        bool valid = fillet.IsDone && fillet.Shape().Validate().IsValid;
        (int lunes, int creases) = asBuilt.InternalBuilder().EnvelopeCornersMade;
        // Its area, beside the script's (OCCT's, whose sections aren't the envelope's: decision 2232) and beside that of
        // the same fillet with every corner tilted, which a corner a few hundredths wide should hardly move
        double area = built ? fillet.Shape().IntegrateArea(new MassOptions { Tolerance = 1e-9 }) : double.NaN;
        MakeFillet allTilted = built ? Fillet(script.Shape, script.Laws, script.Sorted, refused: false, meet: true) : fillet;
        double tilted = allTilted.IsDone ? allTilted.Shape().IntegrateArea(new MassOptions { Tolerance = 1e-9 }) : double.NaN;
        text.AppendLine(string.Create(CultureInfo.InvariantCulture,
            $"AREA\t{name}\t{area:G9}\tOCCT's {script.Area:G9}\t{(area - script.Area) / script.Area:E2}\ttilted {tilted:G9}\t{(area - tilted) / tilted:E2}"));
        text.AppendLine(string.Create(CultureInfo.InvariantCulture,
            $"CASE\t{name}\t{(built ? "built" : $"refused {asBuilt.StripeStatus(1)} ({asBuilt.InternalBuilder().EnvelopeCornerUnbuilt})")}\t{lunes} pieces of the ball, {creases} creases\tdone {fillet.IsDone}\tvalid {valid}\tedge tolerance {Tolerance(fillet):G3}\tedges off their faces {Apart(fillet):G3}\tsurfaces' ends apart {asBuilt.InternalBuilder().EnvelopeJoinWorst:G3}\tworst corner's measure {asBuilt.InternalBuilder().EnvelopeCornerWorst:G4}"));
        List<string> wrong = [];
        double worstRead = double.NaN;
        if (fillet.IsDone)
        {
            List<(double Miss, Point3D At, bool Balls)> points = Union(fillet, script.Shape, out string notes, out List<(Point3D C, double R)[]> lines);
            text.AppendLine($"NOTE\t{name}\t{notes}");

            // The edges between two new faces that aren't tangent across them (unit 15AK3), each by the corner it is at:
            // a crease that was built, a join where the later surface was tilted to end on the earlier's circle, or
            // neither. At the point of each where the faces' normals are furthest apart: that angle; the angle between
            // the two balls of the union the point is on (the boundary of the union has the crease itself, and a built
            // crease has to have its angle); and how far each face's normal is from the radius of a ball that touches
            // the point (a face of the swept surface is tangent to a ball there)
            int creased = 0;
            foreach ((double faces, double union, double off1, double off2, Point3D at, Point3D middle) in Creases(fillet, lines))
            {
                // A crease that was made is the builder's own curve; any other is put to the corner nearest its middle,
                // whose arc's middle that is, within a tenth of the ball's radius
                bool made = fillet.InternalBuilder().EnvelopeCreases.Any(c => c.Value(0.5).DistanceTo(middle) < 1e-6);
                var corner = fillet.InternalBuilder().EnvelopeCorners.OrderBy(c => c.At.DistanceTo(middle)).FirstOrDefault();
                bool atCorner = fillet.InternalBuilder().EnvelopeCorners.Count > 0 && corner.At.DistanceTo(middle) < Math.Max(3 * corner.Gap, 0.1 * corner.Radius);
                string kind = made ? "CREASE" : atCorner ? "JOIN" : "EDGE";
                creased += made ? 1 : 0;
                const double Degrees = 180 / Math.PI;
                text.AppendLine(string.Create(CultureInfo.InvariantCulture,
                    $"{kind}\t{name}\tthe faces' normals {faces * Degrees:F3} degrees apart\tthe union's balls' {union * Degrees:F3}\tthe faces' from a ball's radius {off1 * Degrees:F3} and {off2 * Degrees:F3}\tat ({at.X:F3},{at.Y:F3},{at.Z:F3})\t{(atCorner ? string.Create(CultureInfo.InvariantCulture, $"corner's measure {corner.Off:G3}, gap {corner.Gap:G3}") : "no corner")}"));
                // A built crease's faces are each tangent to a ball that touches there; and where the two balls are told
                // apart (a shallow crossing's are within 2e-6 of each other all the way between), their angle is the faces'
                if (made && !(Math.Max(off1, off2) < 0.02 * faces + 2e-4 && (double.IsNaN(union) || union < 0.5 * faces || Math.Abs(faces - union) < 0.03 * faces + 2e-4)))
                {
                    wrong.Add(string.Create(CultureInfo.InvariantCulture, $"a crease of {faces:G4} radians at ({at.X:F3},{at.Y:F3},{at.Z:F3}) where the union of the balls has one of {union:G4}, its faces {off1:G3} and {off2:G3} from the balls' radii"));
                }
            }

            if (built && creased != asBuilt.InternalBuilder().EnvelopeCornersMade.Creases)
            {
                wrong.Add($"{creased} creases found for {asBuilt.InternalBuilder().EnvelopeCornersMade.Creases} made");
            }

            foreach ((Point3D at, double off, double gap, double radius) in fillet.InternalBuilder().EnvelopeCorners)
            {
                double reach = Math.Max(3 * gap, 0.05);
                var near = points.Where(p => p.At.DistanceTo(at) < reach).ToList();
                // Read where every point beside it is, and no other corner is as near: a reading beside two is of neither
                bool read = near.Count(p => p.Balls) > 0 && near.All(p => p.Balls)
                            && fillet.InternalBuilder().EnvelopeCorners.Count(c => c.Off > 0 && c.At.DistanceTo(at) < reach) <= 1;
                double measured = read ? near.Max(p => p.Miss) : double.NaN;
                text.AppendLine(string.Create(CultureInfo.InvariantCulture,
                    $"CORNER\t{name}\tmeasure {off:G4}\tmeasured {measured:G4}\tgap {gap:G3}\tradius {radius:G4}\tat ({at.X:F3},{at.Y:F3},{at.Z:F3})\t{near.Count(p => p.Balls)} of {near.Count} points read"));
                // A reading says the measure is what the faces are off by, or more: no less than them but for a tenth and
                // the approximation's own error. Past ten times the tolerance, where a corner is a tenth of the radius wide
                // and the second order no longer holds (the refused K5 and K7), only that both are past it
                if (read && (off < 10 * Approximation ? measured > 1.1 * off + 1.5e-5 : measured < Approximation))
                {
                    wrong.Add(string.Create(CultureInfo.InvariantCulture, $"({at.X:F3},{at.Y:F3},{at.Z:F3}): measured {measured:G4}, the measure {off:G4}"));
                }
            }

            double worst = worstRead = points.Where(p => p.Balls).Select(p => p.Miss).DefaultIfEmpty(double.NaN).Max();
            foreach ((double miss, Point3D at, bool _) in points.Where(p => p.Balls && p.Miss > 5e-5).OrderByDescending(p => p.Miss).Take(400))
            {
                text.AppendLine(string.Create(CultureInfo.InvariantCulture, $"PT	{name}	{miss:G4}	{at.X:F4}	{at.Y:F4}	{at.Z:F4}"));
            }
            text.AppendLine(string.Create(CultureInfo.InvariantCulture, $"WORST\t{name}\t{worst:G4}\tof {points.Count(p => p.Balls)} points read, {points.Count(p => !p.Balls)} not"));
        }

        lock (typeof(EvolvingFilletCorpusUnionTests))
        {
            File.AppendAllText(Path.Combine(directory, "union.tsv"), text.ToString());
        }

        output.WriteLine(text.ToString());
        Assert.Empty(wrong);
        Assert.True(!built || valid, $"{name}: built, and not a valid shape");
        // The script's own check, to its tolerance (checkprops: 1%): OCCT's area. And the tilted fillet's, to 1e-4
        Assert.True(!built || Math.Abs(area - script.Area) < 1e-2 * script.Area, $"{name}: built, with an area of {area} for OCCT's {script.Area}");
        Assert.True(!built || double.IsNaN(tilted) || Math.Abs(area - tilted) < 1e-4 * tilted, $"{name}: built, with an area of {area} for {tilted} with its corners tilted");
        Assert.True(!built || Apart(fillet) < Approximation, $"{name}: built, and an edge of a new face is {Apart(fillet)} from a face it bounds");

        // Nothing built is off the union of its balls past the tolerance, wherever it was read; whatever is refused is,
        // somewhere, with the refusal off
        if (fillet.IsDone && !double.IsNaN(worstRead))
        {
            Assert.True(built ? worstRead < Approximation : worstRead > Approximation, $"{name}: {(built ? "built" : "refused")}, off the union of its balls by {worstRead}");
        }
    }

    // Unit 15AK3's cost: each case built with its corners past the tolerance built, against the same case with the
    // refusal off (every corner tilted, as unit 15AK2 built it for its tests), the two alternated seven times in one process,
    // warm: the process's processor time over five builds and the bytes of one. Runs where KERF_15AK3_COST names a file.
    [Fact]
    public void What_building_the_corners_costs()
    {
        if (Environment.GetEnvironmentVariable("KERF_15AK3_COST") is not { Length: > 0 } file)
        {
            return;
        }

        System.Text.StringBuilder text = new();
        foreach (string name in new[] { "I1", "I2", "I3", "I4", "I5", "I7", "I8", "J1", "J2", "J4", "J5", "J7", "J8", "J9", "K1", "K3", "K4", "K5", "K6", "K7" })
        {
            if (Script(name) is not { } script)
            {
                continue;
            }

            MakeFillet Build(bool corners) => Fillet(script.Shape, script.Laws, script.Sorted, refused: corners, meet: true);
            if (!Build(true).IsDone || !Build(false).IsDone || Build(true).InternalBuilder().EnvelopeCornersMade == (0, 0))
            {
                continue;
            }

            List<double>[] times = [[], []], bytes = [[], []];
            for (int round = 0; round < 7; round++)
            {
                for (int form = 0; form < 2; form++)
                {
                    long before = GC.GetAllocatedBytesForCurrentThread();
                    Build(form == 1);
                    bytes[form].Add((GC.GetAllocatedBytesForCurrentThread() - before) / 1048576.0);
                    TimeSpan start = System.Diagnostics.Process.GetCurrentProcess().TotalProcessorTime;
                    for (int k = 0; k < 5; k++)
                    {
                        Build(form == 1);
                    }

                    times[form].Add((System.Diagnostics.Process.GetCurrentProcess().TotalProcessorTime - start).TotalMilliseconds / 5);
                }
            }

            static double Median(List<double> values) => values.Order().ElementAt(values.Count / 2);
            (int lunes, int creases) = Build(true).InternalBuilder().EnvelopeCornersMade;
            text.AppendLine(string.Create(CultureInfo.InvariantCulture,
                $"COST\t{name}\t{lunes} pieces, {creases} creases\ttilted {Median(times[0]):F1} ms ({times[0].Min():F1} to {times[0].Max():F1}), {Median(bytes[0]):F2} MB\tbuilt {Median(times[1]):F1} ms ({times[1].Min():F1} to {times[1].Max():F1}), {Median(bytes[1]):F2} MB"));
        }

        File.WriteAllText(file, text.ToString());
        output.WriteLine(text.ToString());
    }





    // The most a curve of an edge of the new faces is from a surface it bounds, at 41 points of its pcurve there: what the
    // faces are really apart by along their edges, whatever tolerance the edges and their vertices were given
    private static double Apart(MakeFillet fillet)
    {
        if (!fillet.IsDone)
        {
            return double.NaN;
        }

        Face[] all = [.. fillet.Shape().AllFaces()];
        double worst = 0;
        for (int i = 1; i <= fillet.SurfaceCount; i++)
        {
            foreach (Shape shape in fillet.NewFaces(i))
            {
                foreach (Edge e in shape.AllEdges().Where(e => !Tool.Degenerated(e)))
                {
                    var curve = new EdgeCurveAdaptor(e);
                    foreach (Face face in all.Where(g => g.AllEdges().Any(x => x.IsSame(e))))
                    {
                        if (Tool.CurveOnSurface(e, face, out _, out _) is not { } pcurve)
                        {
                            continue;
                        }

                        var surface = new FaceSurfaceAdaptor(face);
                        for (int k = 0; k <= 40; k++)
                        {
                            double t = curve.FirstParameter() + k / 40.0 * (curve.LastParameter() - curve.FirstParameter());
                            Point2D uv = pcurve.Value(t);
                            worst = Math.Max(worst, surface.Value(uv.X, uv.Y).DistanceTo(curve.Value(t)));
                        }
                    }
                }
            }
        }

        return worst;
    }

    // Each of 69 by 69 points of each new face, crowded towards its ends: how far inside the union of the balls or outside
    // it, where it is, and whether that face had balls of its own (a face without them is beside balls that are missing)
    private static List<(double Miss, Point3D At, bool Balls)> Union(MakeFillet fillet, Shape input, out string notes, out List<(Point3D C, double R)[]> lines)
    {
        Face[] inputFaces = [.. input.AllFaces()];
        Edge[] inputEdges = [.. input.AllEdges()];
        List<(Point3D C, double R)[]> balls = [];
        lines = balls;
        List<(FaceSurfaceAdaptor Surface, Face Face)> faces = [];
        HashSet<int> withBalls = [];
        Dictionary<int, string> without = [];
        System.Text.StringBuilder note = new();
        for (int i = 1; i <= fillet.SurfaceCount; i++)
        {
            foreach (Shape shape in fillet.NewFaces(i))
            {
                Face face = (Face)shape;
                var surface = new FaceSurfaceAdaptor(face);
                faces.Add((surface, face));
                double u0 = surface.FirstUParameter(), u1 = surface.LastUParameter(), v0 = surface.FirstVParameter(), v1 = surface.LastVParameter();
                List<(bool AlongV, bool Low, Edge Edge)> onEdges = [];
                foreach ((bool alongV, bool low) in new[] { (true, true), (true, false), (false, true), (false, false) })
                {
                    (double U, double V) At(double t, double across) => alongV
                        ? (low ? u0 + (u1 - u0) * across : u1 - (u1 - u0) * across, v0 + (v1 - v0) * t)
                        : (u0 + (u1 - u0) * t, low ? v0 + (v1 - v0) * across : v1 - (v1 - v0) * across);
                    Point3D On(double t, double across)
                    {
                        (double u, double v) = At(t, across);
                        return surface.Value(u, v);
                    }

                    // The face of the input this boundary lies on, tangent to it. At a stripe's end the surface runs on past
                    // the faces that bound it, so a boundary may leave the face at either end (unit 15AK3: the corpus's I2
                    // starts 0.2 outside the shape); it has to be on it over at least half its length, in one stretch
                    Face? on = null;
                    foreach (Face g in inputFaces)
                    {
                        var gs = new FaceSurfaceAdaptor(g);
                        int firstOn = 0, lastOn = 0, onCount = 0;
                        for (int k = 1; k < 20; k++)
                        {
                            if (!Foot(On(k / 20.0, 0), g, out double d, out double gu, out double gv, out bool inside) || d > 1e-4 || !inside)
                            {
                                continue;
                            }

                            (double u, double v) = At(k / 20.0, 0);
                            surface.D1(u, v, out _, out Vector3D du, out Vector3D dv);
                            gs.D1(gu, gv, out _, out Vector3D gdu, out Vector3D gdv);
                            Vector3D n = du.Crossed(dv), gn = gdu.Crossed(gdv);
                            if (n.Magnitude > 1e-12 && gn.Magnitude > 1e-12 && n.Normalized().Crossed(gn.Normalized()).Magnitude < 0.02)
                            {
                                firstOn = onCount == 0 ? k : firstOn;
                                lastOn = k;
                                onCount++;
                            }
                        }

                        if (onCount >= 10 && lastOn - firstOn + 1 == onCount)
                        {
                            on = g;
                            break;
                        }
                    }

                    if (on is not { } support)
                    {
                        // Or the edge it lies on
                        foreach (Edge e in inputEdges)
                        {
                            if (Enumerable.Range(1, 9).All(k => OnEdge(On(k / 10.0, 0), e, out double d, out _) && d < 1e-4))
                            {
                                onEdges.Add((alongV, low, e));
                                break;
                            }
                        }

                        continue;
                    }

                    var supportSurface = new FaceSurfaceAdaptor(support);
                    bool Normal(double t, out Point3D q, out Vector3D m)
                    {
                        q = On(t, 0);
                        m = default;
                        if (!Foot(q, support, out _, out double gu, out double gv, out bool inside) || !inside)
                        {
                            return false;
                        }

                        supportSurface.D1(gu, gv, out _, out Vector3D gdu, out Vector3D gdv);
                        m = gdu.Crossed(gdv).Normalized();
                        if (m.Dot(new Vector3D(q, On(t, 0.1))) < 0)
                        {
                            m = -m;
                        }

                        return true;
                    }

                    // The other face the ball comes to rest on: the one that puts the far boundary on the ball
                    Face? other = null;
                    int most = 0;
                    double least = double.MaxValue;
                    foreach (Face h in inputFaces)
                    {
                        if (h.IsSame(support))
                        {
                            continue;
                        }

                        var toH = new DistanceShapeShape();
                        toH.LoadS2(h);
                        int ok = 0;
                        double sum = 0;
                        for (int k = 1; k <= 11; k++)
                        {
                            double t = (k - 0.5) / 11;
                            if (!Normal(t, out Point3D q, out Vector3D m))
                            {
                                continue;
                            }

                            Point3D far = On(t, 1);
                            double lo = 0, hi = 4 * q.DistanceTo(far) + 1e-3;
                            if (!(Distance(toH, q.Translated(hi * m)) < hi))
                            {
                                continue;
                            }

                            for (int iter = 0; iter < 40; iter++)
                            {
                                double mid = (lo + hi) / 2;
                                if (Distance(toH, q.Translated(mid * m)) > mid - 1e-6)
                                {
                                    lo = mid;
                                }
                                else
                                {
                                    hi = mid;
                                }
                            }

                            double r = (lo + hi) / 2, error = Math.Abs(q.Translated(r * m).DistanceTo(far) - r);
                            if (error < 2e-3 * r)
                            {
                                ok++;
                                sum += error;
                            }
                        }

                        if (ok >= 9 && (ok > most || (ok == most && sum < least)))
                        {
                            other = h;
                            most = ok;
                            least = sum;
                        }
                    }

                    if (other is not { } rest)
                    {
                        note.Append("[no other face] ");
                        continue;
                    }

                    var toRest = new DistanceShapeShape();
                    toRest.LoadS2(rest);
                    var line = new (Point3D C, double R)[Exact + 1];
                    double last = 0;
                    int bad = 0;
                    for (int k = 0; k <= Exact; k++)
                    {
                        double t = k / (double)Exact;
                        line[k] = (default, double.NaN);
                        if (!Normal(t, out Point3D q, out Vector3D m))
                        {
                            bad++;
                            continue;
                        }

                        Point3D far = On(t, 1);
                        double F(double r) => Distance(toRest, q.Translated(r * m)) - r + 1e-6;
                        double lo, hi, flo, fhi;
                        if (last <= 0)
                        {
                            lo = 0;
                            hi = 4 * q.DistanceTo(far) + 1e-3;
                            flo = F(lo);
                            fhi = F(hi);
                        }
                        else
                        {
                            double step = 0.02 * last;
                            lo = last - step;
                            hi = last + step;
                            flo = F(lo);
                            fhi = F(hi);
                            for (int grow = 0; grow < 12 && (flo < 0 || fhi > 0); grow++)
                            {
                                step *= 2;
                                if (flo < 0)
                                {
                                    lo = Math.Max(0, last - step);
                                    flo = F(lo);
                                }

                                if (fhi > 0)
                                {
                                    hi = last + step;
                                    fhi = F(hi);
                                }
                            }
                        }

                        if (!(flo >= 0 && fhi <= 0))
                        {
                            bad++;
                            continue;
                        }

                        // Illinois
                        int side = 0;
                        for (int iter = 0; iter < 60 && hi - lo > 1e-11; iter++)
                        {
                            double mid = flo - fhi > 0 ? (lo * -fhi + hi * flo) / (flo - fhi) : (lo + hi) / 2;
                            if (!(mid > lo && mid < hi))
                            {
                                mid = (lo + hi) / 2;
                            }

                            double fm = F(mid);
                            if (fm >= 0)
                            {
                                lo = mid;
                                flo = fm;
                                if (side == 1)
                                {
                                    fhi /= 2;
                                }

                                side = 1;
                            }
                            else
                            {
                                hi = mid;
                                fhi = fm;
                                if (side == -1)
                                {
                                    flo /= 2;
                                }

                                side = -1;
                            }
                        }

                        double radius = (lo + hi) / 2;
                        last = radius;
                        Point3D c = q.Translated(radius * m);
                        if (Math.Abs(c.DistanceTo(far) - radius) > 2e-3 * radius)
                        {
                            bad++;
                            continue;
                        }

                        line[k] = (c, radius);
                    }

                    if (Carry(line) is { } dense)
                    {
                        balls.Add(dense);
                        withBalls.Add(faces.Count - 1);
                    }
                    else
                    {
                        note.Append($"[a boundary left out: {bad} of its balls not found] ");
                    }
                }

                without[faces.Count - 1] = $"{onEdges.Count} boundaries on edges{(onEdges.Count == 2 && onEdges[0].AlongV != onEdges[1].AlongV ? ", not opposite" : "")}";

                // Between two edges: the ball through the two points whose chords to them are square to the edges
                if (!withBalls.Contains(faces.Count - 1) && onEdges.Count == 2 && onEdges[0].AlongV == onEdges[1].AlongV)
                {
                    bool alongV = onEdges[0].AlongV;
                    Point3D On(double t, bool low) => alongV ? surface.Value(low ? u0 : u1, v0 + (v1 - v0) * t) : surface.Value(u0 + (u1 - u0) * t, low ? v0 : v1);
                    Edge e1 = onEdges.Single(o => o.Low).Edge, e2 = onEdges.Single(o => !o.Low).Edge;
                    EdgeCurveAdaptor c1 = new(e1), c2 = new(e2);
                    var line = new (Point3D C, double R)[Exact + 1];
                    int fromSection = 0;
                    for (int k = 0; k <= Exact; k++)
                    {
                        double t = k / (double)Exact;
                        Point3D p1 = On(t, true), p2 = On(t, false);
                        line[k] = (default, double.NaN);
                        if (!OnEdge(p1, e1, out _, out double s1) || !OnEdge(p2, e2, out _, out double s2))
                        {
                            continue;
                        }

                        c1.D1(s1, out _, out Vector3D t1);
                        c2.D1(s2, out _, out Vector3D t2);
                        // (c - p1) . t1 = 0, (c - p2) . t2 = 0, |c - p1| = |c - p2|, in x = c - p1
                        Vector3D chord = new(p1, p2), n1 = t1.Normalized(), n2 = t2.Normalized();
                        double det = n1.Dot(n2.Crossed(chord));
                        // The three planes that fix the centre nearly share a line where the edges and the chord are nearly
                        // in one plane (two edges of one flat face: the contacts then say where the centre is over that
                        // plane and not how high). There the ball's size is taken from the face's own section: the ball
                        // also goes through the middle of the line across the face between the two contacts, a fourth
                        // plane, and the centre is the least-squares point of the four (unit 15AK3). A ball so read is
                        // the surface's own at that section, so the reading beside it shows only whether the face is
                        // the envelope of its sections' balls, not whether those are of the radius asked.
                        if (Math.Abs(det) < 0.05 * chord.Magnitude)
                        {
                            Point3D middle = alongV ? surface.Value((u0 + u1) / 2, v0 + (v1 - v0) * t) : surface.Value(u0 + (u1 - u0) * t, (v0 + v1) / 2);
                            Vector3D toMiddle = new(p1, middle);
                            (Vector3D Row, double Right)[] rows =
                            [
                                (n1, 0.0), (n2, chord.Dot(n2)), (chord.Normalized(), chord.Magnitude / 2), (toMiddle.Normalized(), toMiddle.Magnitude / 2),
                            ];
                            double[,] a = new double[3, 3];
                            double[] b = new double[3];
                            foreach ((Vector3D row, double right) in rows)
                            {
                                double[] r = [row.X, row.Y, row.Z];
                                for (int row1 = 0; row1 < 3; row1++)
                                {
                                    b[row1] += r[row1] * right;
                                    for (int row2 = 0; row2 < 3; row2++)
                                    {
                                        a[row1, row2] += r[row1] * r[row2];
                                    }
                                }
                            }

                            double d = a[0, 0] * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1]) - a[0, 1] * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0])
                                       + a[0, 2] * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]);
                            if (Math.Abs(d) < 1e-6)
                            {
                                continue;
                            }

                            double x0 = (b[0] * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1]) - a[0, 1] * (b[1] * a[2, 2] - a[1, 2] * b[2])
                                         + a[0, 2] * (b[1] * a[2, 1] - a[1, 1] * b[2])) / d;
                            double x1 = (a[0, 0] * (b[1] * a[2, 2] - a[1, 2] * b[2]) - b[0] * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0])
                                         + a[0, 2] * (a[1, 0] * b[2] - b[1] * a[2, 0])) / d;
                            double x2 = (a[0, 0] * (a[1, 1] * b[2] - b[1] * a[2, 1]) - a[0, 1] * (a[1, 0] * b[2] - b[1] * a[2, 0])
                                         + b[0] * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0])) / d;
                            Vector3D fitted = new(x0, x1, x2);
                            line[k] = (p1.Translated(fitted), fitted.Magnitude);
                            fromSection++;
                            continue;
                        }

                        double b2 = chord.Dot(n2), b3 = chord.SquareMagnitude / 2;
                        Vector3D x = (b2 * chord.Crossed(n1) + b3 * n1.Crossed(n2)) / det;
                        line[k] = (p1.Translated(x), x.Magnitude);
                    }

                    if (Carry(line) is { } dense)
                    {
                        balls.Add(dense);
                        withBalls.Add(faces.Count - 1);
                        note.Append(fromSection > 0 ? $"[between two edges, {fromSection} of {Exact + 1} balls' sizes from the face's section] " : "[between two edges] ");
                    }
                }
            }
        }

        for (int index = 0; index < faces.Count; index++)
        {
            if (!withBalls.Contains(index) && !faces[index].Face.AllEdges().Any(e => Tool.Degenerated(e)))
            {
                note.Append($"[face {index + 1} has no balls: {without[index]}] ");
            }
        }

        // A piece of the ball at a corner (unit 15AK3) touches the input at two points only: it is read against its
        // neighbours' balls, of which the corner's is the last
        int pieces = 0;
        for (int index = 0; index < faces.Count; index++)
        {
            if (!withBalls.Contains(index) && faces[index].Face.AllEdges().Any(e => Tool.Degenerated(e)))
            {
                withBalls.Add(index);
                pieces++;
            }
        }

        note.Append($"{balls.Count} lines of balls, {withBalls.Count - pieces} of {faces.Count} faces with their own, {pieces} pieces of the ball");
        notes = note.ToString();
        List<(double, Point3D, bool)> points = [];
        double[] near = [1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 2e-3, 4e-3, 7e-3, 0.012, 0.02, 0.03, 0.045, 0.06, 0.08, 0.1, 0.13, 0.16, 0.2, 0.25, 0.3, 0.35, 0.4];
        double[] ts = [.. Enumerable.Range(0, 25).Select(k => k / 24.0), .. near, .. near.Select(x => 1 - x)];
        for (int index = 0; index < faces.Count; index++)
        {
            (FaceSurfaceAdaptor surface, Face face) = faces[index];
            double u0 = surface.FirstUParameter(), u1 = surface.LastUParameter(), v0 = surface.FirstVParameter(), v1 = surface.LastVParameter();
            bool own = withBalls.Contains(index);
            foreach (double ta in ts)
            {
                foreach (double tb in ts)
                {
                    double u = u0 + (u1 - u0) * ta, v = v0 + (v1 - v0) * tb;
                    if (new Kerf.Engine.Classification.FaceClassifier(face, new Point2D(u, v), 1e-7).State() != Containment.In)
                    {
                        continue;
                    }

                    Point3D p = surface.Value(u, v);
                    double depth = double.MinValue;
                    bool last = false;
                    foreach ((Point3D C, double R)[] line in own ? balls : [])
                    {
                        // The deepest of every fiftieth ball, then of all the balls about the three best of those
                        Span<(double D, int K)> best = [(double.MinValue, 0), (double.MinValue, 0), (double.MinValue, 0)];
                        for (int k = 0; k <= Dense; k += Coarse)
                        {
                            double d = line[k].R - p.DistanceTo(line[k].C);
                            if (d > best[0].D)
                            {
                                best[2] = best[1];
                                best[1] = best[0];
                                best[0] = (d, k);
                            }
                            else if (d > best[1].D)
                            {
                                best[2] = best[1];
                                best[1] = (d, k);
                            }
                            else if (d > best[2].D)
                            {
                                best[2] = (d, k);
                            }
                        }

                        foreach ((double _, int centre) in best)
                        {
                            for (int k = Math.Max(0, centre - Coarse); k <= Math.Min(Dense, centre + Coarse); k++)
                            {
                                double d = line[k].R - p.DistanceTo(line[k].C);
                                if (d > depth)
                                {
                                    depth = d;
                                    last = k == 0 || k == Dense;
                                }
                            }
                        }
                    }

                    // Outside every ball, the nearest the last of its line: beyond the balls read (a stripe's end, where the
                    // face runs on to the faces that bound it), not a reading
                    bool read = own && !(last && depth < -1e-5);
                    points.Add((read ? Math.Abs(depth) : double.NaN, p, read));
                }
            }
        }

        return points;
    }

    // Each edge between two of the new faces that is a crease the builder made, or whose normals are more than 0.002 radians
    // apart somewhere along it (a tangent join is within 2e-4): the greatest angle between them, at 41 points, where, and the greatest angle there between the
    // radii of the balls that touch the point, each the nearest of a stretch of its line that comes within 2e-6 of it. NaN
    // where fewer than two balls do.
    private static List<(double Faces, double Union, double Off1, double Off2, Point3D At, Point3D Middle)> Creases(MakeFillet fillet, List<(Point3D C, double R)[]> lines)
    {
        List<Face> faces = [];
        for (int i = 1; i <= fillet.SurfaceCount; i++)
        {
            faces.AddRange(fillet.NewFaces(i).Select(f => (Face)f));
        }

        List<(double, double, double, double, Point3D, Point3D)> creases = [];
        List<Edge> seen = [];
        foreach (Face face in faces)
        {
            foreach (Edge e in face.AllEdges().Where(e => !Tool.Degenerated(e)))
            {
                if (seen.Any(x => x.IsSame(e)))
                {
                    continue;
                }

                seen.Add(e);
                Face[] both = [.. faces.Where(g => g.AllEdges().Any(x => x.IsSame(e)))];
                if (both.Length != 2 || Tool.CurveOnSurface(e, both[0], out _, out _) is not { } first || Tool.CurveOnSurface(e, both[1], out _, out _) is not { } second)
                {
                    continue;
                }

                var curve = new EdgeCurveAdaptor(e);
                FaceSurfaceAdaptor s1 = new(both[0]), s2 = new(both[1]);
                double greatest = 0;
                Point3D at = default;
                Vector3D normal1 = default, normal2 = default;
                for (int k = 0; k <= 40; k++)
                {
                    double t = curve.FirstParameter() + k / 40.0 * (curve.LastParameter() - curve.FirstParameter());
                    Point2D a = first.Value(t), b = second.Value(t);
                    s1.D1(a.X, a.Y, out _, out Vector3D du1, out Vector3D dv1);
                    s2.D1(b.X, b.Y, out _, out Vector3D du2, out Vector3D dv2);
                    Vector3D n1 = du1.Crossed(dv1), n2 = du2.Crossed(dv2);
                    if (n1.Magnitude < 1e-12 || n2.Magnitude < 1e-12)
                    {
                        continue;
                    }

                    double angle = Math.Asin(Math.Min(1.0, n1.Normalized().Crossed(n2.Normalized()).Magnitude));
                    if (angle > greatest)
                    {
                        greatest = angle;
                        at = curve.Value(t);
                        normal1 = n1.Normalized();
                        normal2 = n2.Normalized();
                    }
                }

                Point3D half = curve.Value(0.5 * (curve.FirstParameter() + curve.LastParameter()));
                if (greatest < 2e-3 && !fillet.InternalBuilder().EnvelopeCreases.Any(c => c.Value(0.5).DistanceTo(half) < 1e-6))
                {
                    continue;
                }

                List<Vector3D> radii = [];
                foreach ((Point3D C, double R)[] line in lines)
                {
                    int best = -1;
                    double nearest = double.MinValue;
                    for (int k = 0; k <= Dense + 1; k++)
                    {
                        double depth = k <= Dense ? line[k].R - at.DistanceTo(line[k].C) : double.MinValue;
                        if (depth > -2e-6)
                        {
                            if (depth > nearest)
                            {
                                nearest = depth;
                                best = k;
                            }
                        }
                        else if (best >= 0)
                        {
                            radii.Add(new Vector3D(line[best].C, at).Normalized());
                            best = -1;
                            nearest = double.MinValue;
                        }
                    }
                }

                double union = double.NaN;
                for (int i = 0; i < radii.Count; i++)
                {
                    for (int j = i + 1; j < radii.Count; j++)
                    {
                        double angle = Math.Asin(Math.Min(1.0, radii[i].Crossed(radii[j]).Magnitude));
                        union = double.IsNaN(union) ? angle : Math.Max(union, angle);
                    }
                }

                double From(Vector3D normal) => radii.Count == 0 ? double.NaN : radii.Min(r => Math.Asin(Math.Min(1.0, r.Crossed(normal).Magnitude)));
                creases.Add((greatest, union, From(normal1), From(normal2), at, curve.Value(0.5 * (curve.FirstParameter() + curve.LastParameter()))));
            }
        }

        return creases;
    }

    // The balls found along a boundary, less those at its ends that weren't, carried to 30,001 by the cubic through the four
    // nearest; none where one inside is missing or too few are left
    private static (Point3D C, double R)[]? Carry((Point3D C, double R)[] line)
    {
        int from = 0, to = line.Length - 1;
        while (from < to && double.IsNaN(line[from].R))
        {
            from++;
        }

        while (to > from && double.IsNaN(line[to].R))
        {
            to--;
        }

        if (to - from < line.Length / 2 || Enumerable.Range(from, to - from + 1).Any(k => double.IsNaN(line[k].R)))
        {
            return null;
        }

        int exact = to - from;
        var dense = new (Point3D C, double R)[Dense + 1];
        for (int j = 0; j <= Dense; j++)
        {
            double s = j / (double)Dense * exact;
            int i1 = Math.Clamp((int)Math.Floor(s), 1, exact - 2);
            double x = s - i1;
            double w0 = -x * (x - 1) * (x - 2) / 6, w1 = (x + 1) * (x - 1) * (x - 2) / 2, w2 = -(x + 1) * x * (x - 2) / 2, w3 = (x + 1) * x * (x - 1) / 6;
            (Point3D a, double ra) = line[from + i1 - 1];
            (Point3D b, double rb) = line[from + i1];
            (Point3D c, double rc) = line[from + i1 + 1];
            (Point3D d, double rd) = line[from + i1 + 2];
            dense[j] = (new Point3D(w0 * a.X + w1 * b.X + w2 * c.X + w3 * d.X, w0 * a.Y + w1 * b.Y + w2 * c.Y + w3 * d.Y,
                                    w0 * a.Z + w1 * b.Z + w2 * c.Z + w3 * d.Z), w0 * ra + w1 * rb + w2 * rc + w3 * rd);
        }

        return dense;
    }

    private static double Distance(DistanceShapeShape to, Point3D p)
    {
        to.LoadS1(new MakeVertex(p).Shape());
        to.Perform();
        return to.IsDone ? to.Value() : double.NaN;
    }

    // The nearest point of the face to q: how far, its parameters, and whether it is inside the face
    private static bool Foot(Point3D q, Face g, out double distance, out double u, out double v, out bool inside)
    {
        var d = new DistanceShapeShape();
        d.LoadS1(new MakeVertex(q).Shape());
        d.LoadS2(g);
        d.Perform();
        distance = double.MaxValue;
        u = v = 0;
        inside = false;
        if (!d.IsDone || d.SolutionCount < 1)
        {
            return false;
        }

        distance = d.Value();
        inside = d.SupportTypeShape2(1) == SupportType.IsInFace;
        if (inside)
        {
            d.ParOnFaceS2(1, out u, out v);
        }

        return true;
    }

    // The nearest point of the edge to q, inside it: how far, and its parameter
    private static bool OnEdge(Point3D q, Edge e, out double distance, out double parameter)
    {
        var d = new DistanceShapeShape();
        d.LoadS1(new MakeVertex(q).Shape());
        d.LoadS2(e);
        d.Perform();
        distance = double.MaxValue;
        parameter = 0;
        if (!d.IsDone || d.SolutionCount < 1 || d.SupportTypeShape2(1) != SupportType.IsOnEdge)
        {
            return false;
        }

        distance = d.Value();
        d.ParOnEdgeS2(1, out parameter);
        return true;
    }
}
