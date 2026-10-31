#if HAS_ORACLE || HAS_NATIVE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Build123d;
using static Build123d.Operations;
using B = Build123d;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The Booleans and clean-ups build123d.NET's own tests make (unit 6M, the phase 6 milestone), rebuilt with build123d.NET's
/// public API as open question 20's provisional answer has it (decision 7501): each scenario transcribes one test's
/// construction, and the capture records every <c>BRepAlgoAPI_*</c> call build123d.NET's Boolean path makes on the way
/// (its arguments and tools as the native shapes it hands OCCT, its options) and every <c>ShapeUpgrade_UnifySameDomain</c>
/// it runs (<c>Clean()</c>, and the clean after each Boolean). The scenario goes on with build123d.NET's own result.
/// </summary>
/// <remarks>
/// The capture mirrors build123d.NET's dispatch (Shape.Booleans.cs, Compound.cs, Shape2D.cs, Shape3D.cs, Shape1D.cs):
/// <list type="bullet">
/// <item><c>a - b</c> (Subtract): a cut of <c>[a]</c> by the top-level shapes of <c>b</c>;</item>
/// <item><c>a + b</c> (Add): for a Part, Sketch or plain compound, a fuse of its first top-level shape with the rest of its
/// own and <c>b</c>'s, fuzzy at <c>Constants.Tolerance</c>; for a solid or face, a fuse of <c>[a]</c> with <c>b</c>'s
/// top-level shapes; joining 1D shapes (wires) and sewing shells make no Boolean;</item>
/// <item><c>a &amp; b</c> and <c>Intersect</c>: per element, a common for a solid, a common and a section for two faces or
/// shells, a section for a face and an edge, a common and a section for two edges; then a clean of the result;</item>
/// <item><c>Fuse</c> and <c>Cut</c>: <c>[a]</c> against the shapes given, with glue (shift) and the fuzzy value;</item>
/// <item><c>Split</c>: a splitter of <c>[a]</c> by the plane's face or the tool, not cleaned.</item>
/// </list>
/// Every Boolean runs in parallel and non-destructively, as build123d.NET's does. Operations that clean or fuse inside
/// (Extrude both ways, Revolve, Loft, Section, Thicken, Trace) are transcribed from their source with their public building
/// blocks (<c>Solid.Extrude</c>, <c>clean: false</c>...). The touches <c>includeTouched</c> adds are left out: they repeat
/// the same Booleans.
/// </remarks>
internal sealed class Build123dBooleanCapture
{
    internal enum Kind { Fuse, Cut, Common, Section, Split, Clean }

    /// <summary>
    /// One native call: a Boolean (its arguments, tools and options, and whether build123d.NET cleans its result) or a
    /// clean (its shape in <see cref="Arguments"/>). <see cref="Result"/> is build123d.NET's own result of an operator or
    /// method that makes just this Boolean (and its clean), which checks the transcription.
    /// </summary>
    internal sealed record Call(string Name, Kind Kind, N.TopoDS_Shape[] Arguments, N.TopoDS_Shape[] Tools, double Fuzzy, bool Glue,
                                bool Clean, N.TopoDS_Shape? Result = null);

    internal readonly List<Call> Calls = [];

    private string myScenario = "";
    private int myIndex;

    /// <summary>Runs every scenario, each recording its calls under its name.</summary>
    internal static Build123dBooleanCapture Capture(Action<string>? theFailure = null)
    {
        var aCapture = new Build123dBooleanCapture();
        foreach ((string aName, Action<Build123dBooleanCapture> aScenario) in Scenarios())
        {
            aCapture.myScenario = aName;
            aCapture.myIndex = 0;
            try
            {
                aScenario(aCapture);
            }
            catch (Exception e)
            {
                theFailure?.Invoke($"{aName}: {e.GetType().Name}: {e.Message}");
            }
        }
        return aCapture;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // build123d.NET's Boolean path

    private void Record(Kind theKind, IEnumerable<B.Shape> theArgs, IEnumerable<B.Shape> theTools, double theFuzzy = 0, bool theGlue = false,
                        bool theClean = true)
    {
        // BoolOp drops empty operands; with no arguments or no tools it makes no native call
        N.TopoDS_Shape[] anArgs = [.. theArgs.Where(s => !s.IsEmpty).Select(s => s.Wrapped)];
        N.TopoDS_Shape[] aTools = [.. theTools.Where(s => !s.IsEmpty).Select(s => s.Wrapped)];
        if (anArgs.Length == 0 || aTools.Length == 0)
        {
            return;
        }
        Calls.Add(new Call($"{myScenario} #{++myIndex}", theKind, anArgs, aTools, theFuzzy, theGlue, theClean && SkipClean.Clean));
    }

    // build123d.NET's result of the call just recorded, if any
    private B.Shape Checked(int theCount, B.Shape theResult)
    {
        if (Calls.Count == theCount + 1 && !theResult.IsEmpty)
        {
            Calls[^1] = Calls[^1] with { Result = theResult.Wrapped };
        }
        return theResult;
    }

    private void RecordClean(B.Shape theShape)
    {
        if (!theShape.IsEmpty)
        {
            Calls.Add(new Call($"{myScenario} #{++myIndex}", Kind.Clean, [theShape.Wrapped], [], 0, false, false));
        }
    }

    // Shape.Operands: the top-level shapes of the operands
    private static List<B.Shape> Operands(IEnumerable<B.Shape?> theOthers) => [.. theOthers.OfType<B.Shape>().SelectMany(o => o.GetTopLevelShapes())];

    private static int? Dim(B.Shape theShape) => theShape switch
    {
        B.Part => 3,
        B.Sketch => 2,
        B.Curve => 1,
        B.Compound c => c.GetTopLevelShapes().Select(Dim).Distinct().ToList() is [int d] ? d : null,
        B.Solid => 3,
        B.Face or B.Shell => 2,
        B.Edge or B.Wire => 1,
        _ => 0,
    };

    /// <summary><c>a - b</c>.</summary>
    internal B.Shape Sub(B.Shape a, params B.Shape?[] b)
    {
        int aCount = Calls.Count;
        Record(Kind.Cut, [a], Operands(b));
        return Checked(aCount, a - b);
    }

    /// <summary><c>a + b</c>.</summary>
    internal B.Shape Add(B.Shape a, params B.Shape?[] b)
    {
        int aCount = Calls.Count;
        List<B.Shape> aBrought = Operands(b);
        if (aBrought.Count > 0)
        {
            if (a is B.Compound && Dim(a) != 1)
            {
                List<B.Shape> aSummands = [.. a.GetTopLevelShapes(), .. aBrought];
                if (aSummands.Count > 1)
                {
                    Record(Kind.Fuse, aSummands.Take(1), aSummands.Skip(1), B.Constants.Tolerance);
                }
            }
            else if (a is B.Solid || (a is B.Face && !aBrought.Any(s => s is B.Shell)))
            {
                if (a.IsEmpty)
                {
                    Record(Kind.Fuse, aBrought.Take(1), aBrought.Skip(1));
                }
                else
                {
                    Record(Kind.Fuse, [a], aBrought);
                }
            }
        }
        return Checked(aCount, a + b);
    }

    /// <summary><c>a.Fuse(b, glue, tolerance)</c>.</summary>
    internal B.Shape Fuse(B.Shape a, IEnumerable<B.Shape> b, bool glue = false, double? tolerance = null)
    {
        int aCount = Calls.Count;
        Record(Kind.Fuse, [a], b, tolerance ?? 0, glue);
        return Checked(aCount, a.Fuse(b, glue, tolerance));
    }

    /// <summary><c>a.Cut(b)</c>.</summary>
    internal B.Shape Cut(B.Shape a, params B.Shape[] b)
    {
        int aCount = Calls.Count;
        Record(Kind.Cut, [a], b);
        return Checked(aCount, a.Cut(b));
    }

    /// <summary><c>a &amp; b</c> (CommonWith): the intersection, then its clean.</summary>
    internal B.Shape And(B.Shape a, B.Shape b)
    {
        B.ShapeList<B.Shape> aFound = Intersect(a, [b]);
        B.Shape aResult = a & b;
        if (aFound.Count > 0 && !aResult.IsEmpty)
        {
            // CommonWith cleans what Intersect found: one shape, or a composite of several
            RecordClean(aFound.Count == 1 ? aFound[0] : new B.Compound(aFound));
        }
        return aResult;
    }

    /// <summary><c>a.Intersect(targets)</c>: chained over the targets.</summary>
    internal B.ShapeList<B.Shape> Intersect(B.Shape a, object[] theTargets, bool theIncludeTouched = false)
    {
        if (a.IsEmpty || theTargets.Length == 0 || theTargets.Any(o => o is B.Shape { IsEmpty: true }))
        {
            return [];
        }
        List<B.Shape> aSet = [a];
        foreach (object aTarget in theTargets)
        {
            var aNext = new List<B.Shape>();
            foreach (B.Shape anObj in aSet)
            {
                IntersectCore(anObj, aTarget);
                aNext.AddRange(anObj.Intersect([aTarget], includeTouched: theIncludeTouched));
            }
            if (aNext.Count == 0)
            {
                break;
            }
            aSet = [.. aNext.Distinct()];
        }
        return a.Intersect(theTargets, includeTouched: theIncludeTouched);
    }

    // IntersectCore's Booleans, by the kinds of the two shapes
    private void IntersectCore(B.Shape theSelf, object theOther)
    {
        if (theSelf is B.Compound aC)
        {
            List<B.Shape> aMine = [.. aC.Children()];
            List<B.Shape> aTheirs = theOther is B.Compound c ? [.. c.Children()] : [B.Shape.AsShape(theOther)];
            foreach (B.Shape x in aMine)
            {
                foreach (B.Shape y in aTheirs)
                {
                    IntersectCore(x, y);
                }
            }
            return;
        }
        if (theSelf is B.Vertex)
        {
            return;
        }
        if (theSelf is B.Edge or B.Wire && theOther is B.Plane aPlane)
        {
            Record(Kind.Section, [theSelf], [B.Face.FromPlane(aPlane)]);
            return;
        }
        B.Shape aShape = B.Shape.AsShape(theOther);
        if (aShape is B.Edge { IsInfinite: true } anInfinite)
        {
            BoundBox aBox = theSelf.BoundingBox(optimal: false);
            aShape = anInfinite.TrimInfinite(aBox.Diagonal + (anInfinite.Center() - aBox.Center()).Length);
        }
        switch (theSelf)
        {
            case B.Solid when aShape is B.Solid or B.Face or B.Shell or B.Edge or B.Wire:
                Record(Kind.Common, [theSelf], [aShape]);
                break;
            case B.Face or B.Shell when aShape is B.Face or B.Shell:
                Record(Kind.Common, [theSelf], [aShape]);
                Record(Kind.Section, [theSelf], [aShape]);
                break;
            case B.Face or B.Shell when aShape is B.Edge or B.Wire:
                Record(Kind.Section, [theSelf], [aShape]);
                break;
            case B.Edge or B.Wire when aShape is B.Edge or B.Wire:
                Record(Kind.Common, [theSelf], [aShape]);
                Record(Kind.Section, [theSelf], [aShape]);
                break;
            case B.Solid or B.Face or B.Shell or B.Edge or B.Wire when aShape is not B.Vertex:
                // a higher-order shape (or a compound) takes over
                IntersectCore(aShape, theSelf);
                break;
        }
    }

    /// <summary><c>a.Split(plane, keep)</c>.</summary>
    internal B.ShapeList<B.Shape> Split(B.Shape a, B.Plane thePlane, Keep theKeep = Keep.Top)
    {
        Record(Kind.Split, [a], [B.Face.FromPlane(thePlane)], theClean: false);
        return a.Split(thePlane, theKeep);
    }

    /// <summary><c>a.Split(tool, keep)</c>: the splitter (the side of each piece is a common with a half-space, left out).</summary>
    internal B.ShapeList<B.Shape> Split(B.Shape a, B.Shape theTool, Keep theKeep = Keep.Top)
    {
        Record(Kind.Split, [a], [theTool], theClean: false);
        return a.Split(theTool, theKeep);
    }

    /// <summary><c>s.Clean()</c>.</summary>
    internal T Clean<T>(T s) where T : B.Shape
    {
        RecordClean(s);
        return s.Clean();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Operations that fuse or clean inside, from their source (Operations/OperationsPart.cs, OperationsSketch.cs)

    /// <summary>Extrude's per-face solids, the fuse of both directions and the clean.</summary>
    internal B.Part Extrude(B.Shape theShape, double theAmount, bool theBoth = false)
    {
        var aSolids = new List<B.Shape>();
        foreach (B.Face aFace in theShape.Faces())
        {
            var aPlane = new B.Plane(aFace);
            foreach (int aSign in theBoth ? new[] { 1, -1 } : [1])
            {
                aSolids.Add(B.Solid.Extrude(aFace, aPlane.ZDir * theAmount * aSign));
            }
        }
        if (theBoth && aSolids.Count > 1)
        {
            aSolids = [Fuse(aSolids[^1], aSolids.Take(aSolids.Count - 1))];
        }
        aSolids = [.. aSolids.Select(Clean)];
        return new B.Part(new B.ShapeList<B.Shape>(aSolids).Solids());
    }

    /// <summary>Revolve's solids, cleaned.</summary>
    internal B.Part Revolve(B.Shape theProfile, B.Axis theAxis, double theAngle = 360)
    {
        var aSolids = theProfile.Faces().Select(f => (B.Shape)B.Solid.Revolve(f, theAngle, theAxis)).ToList();
        return new B.Part(aSolids.Select(Clean));
    }

    /// <summary>Loft, cleaned (its holes, cut away inside, are left to build123d.NET).</summary>
    internal B.Part Loft(IEnumerable<B.Shape> theSections, bool theRuled = false)
    {
        B.Part aRaw = Operations.Loft(theSections, theRuled, clean: false);
        return new B.Part(aRaw.Solids().Select(s => (B.Shape)Clean(s)));
    }

    /// <summary>Section's plane faces, each intersected with the object, then cleaned.</summary>
    internal B.Sketch Section(B.Shape theObj, IEnumerable<B.Plane> theBy, double theHeight = 0)
    {
        BoundBox aBox = theObj.BoundingBox(optimal: false);
        double aMax = new[] { aBox.Min.X, aBox.Min.Y, aBox.Min.Z, aBox.Max.X, aBox.Max.Y, aBox.Max.Z }.Max(Math.Abs) + aBox.Diagonal;
        var aResults = new List<B.Shape>();
        foreach (B.Plane aPlane in theBy)
        {
            B.Face aFace = B.Face.MakeRect(2 * aMax, 2 * aMax, new B.Plane(aPlane.Origin + aPlane.ZDir * theHeight, zDir: aPlane.ZDir));
            aResults.AddRange(Intersect(theObj, [aFace]));
        }
        return new B.Sketch(aResults.Select(Clean));
    }

    /// <summary>Thicken's solids, the fuse of several and the clean.</summary>
    internal B.Part Thicken(B.Shape theShape, double theAmount, bool theBoth = false)
    {
        var aSolids = new List<B.Shape>();
        foreach (B.Face aFace in theShape.Faces())
        {
            B.Vector aNormal = aFace.NormalAt();
            foreach (int aSign in theBoth ? new[] { 1, -1 } : [1])
            {
                aSolids.Add(B.Solid.Thicken(aFace, theAmount, aNormal * aSign));
            }
        }
        if (aSolids.Count > 1)
        {
            aSolids = [Fuse(aSolids[0], aSolids.Skip(1))];
        }
        return new B.Part(aSolids.Select(Clean));
    }

    /// <summary>Trace's swept faces, fused.</summary>
    internal B.Sketch Trace(B.Shape theLines, double theWidth)
    {
        var aFaces = new List<B.Face>();
        foreach (B.Wire aWire in B.Wire.Combine(theLines.Edges()))
        {
            B.Edge aPen = aWire.PerpendicularLine(theWidth, 0);
            aFaces.AddRange(B.Shell.Sweep(aPen, aWire, Transition.Right).Faces());
        }
        B.Shape aCombined = aFaces.Count > 1 ? Fuse(aFaces[0], aFaces.Skip(1)) : aFaces[0];
        return new B.Sketch(aCombined.Wrapped);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The scenarios: build123d.NET's tests (tests/Build123d.Tests), their Booleans in order

    // The tests' helpers (Topology/CompoundTests.cs's CompoundObjects, IntersectionTests.cs's and ShapeTests.cs's B and
    // Objects, Shape3DTests.cs's Box)
    private static B.Part CBox(double l, double w, double h) => new([B.Solid.MakeBox(l, w, h).Moved(new B.Location((-l / 2, -w / 2, -h / 2)))]);
    private static B.Part CCylinder(double r, double h, bool onBase = false) =>
        new([onBase ? B.Solid.MakeCylinder(r, h) : B.Solid.MakeCylinder(r, h).Moved(new B.Location((0, 0, -h / 2)))]);
    private static B.Part CSphere(double r) => new([B.Solid.MakeSphere(r)]);
    private static B.Sketch CRectangle(double w, double h) => new([B.Face.MakeRect(w, h)]);
    private static B.Solid SBox(double l, double w, double h) => B.Solid.MakeBox(l, w, h, new B.Plane((-l / 2, -w / 2, -h / 2)));
    private static B.Solid SBoxMin(double l, double w, double h) => B.Solid.MakeBox(l, w, h);
    private static B.Solid SCylinder(double r, double h) => B.Solid.MakeCylinder(r, h, new B.Plane((0, 0, -h / 2)));
    private static B.Face FRect(double w, double h) => B.Face.MakeRect(w, h);
    private static B.Face FCircle(double r) => new(B.Wire.MakeCircle(r));
    private static B.Part PBox(double l, double w, double h) => new([B.Solid.MakeBox(l, w, h, new B.Plane((-l / 2, -w / 2, -h / 2)))]);

    internal static IEnumerable<(string, Action<Build123dBooleanCapture>)> Scenarios()
    {
        // ---- Operations/OperationsGenericTests.cs ----
        yield return ("Generic.BoundingBoxToSketch", c =>
        {
            var box = BoundingBox(new Circle(10).Faces());
            var ears = (box.Vertices() > Axis.Y)[..^2];
            B.Shape mickey = new Circle(10);
            foreach (var ear in ears)
            {
                mickey = c.Add(mickey, new B.Location(ear.ToVector()) * new Circle(7));
            }
        });
        yield return ("Generic.SketchChamfer", c => c.Add(new Pos(-10, 0) * new Rectangle(10, 10), new Pos(10, 0) * new Rectangle(10, 10)));
        yield return ("Generic.SketchChamferWithHole", c => c.Sub(new Rectangle(20, 20), new Rectangle(5, 5)));
        yield return ("Generic.MirrorChangingObjectType", c =>
        {
            double ringR = 9, ringT = 2, wheelR = 10, wheelT = 6;
            var side = c.Sub(c.Sub(new Trapezoid(wheelR, wheelT / 2, 90, 45, align: (Align.Min, Align.Min)),
                                   new Pos(ringR, ringT / 2) * new Circle(ringT / 2)),
                             new Pos(wheelR, ringT / 2) * new Rectangle(2, 2));
            c.Revolve(Plane.XZ * side, Axis.Z);
        });
        yield return ("Generic.FaceOffsetWithHoles", c => c.Sub(new Rectangle(100, 100), [.. new GridLocations(80, 80, 2, 2) * new Circle(5)]));
        yield return ("Generic.OffsetCurve", c =>
        {
            B.Shape s = Offset(Edge.MakeBezier((0, 0), (1, 5), (0, 10)), 1);
            c.Sub(s, new Rectangle(100, 100, align: (Align.Max, Align.Center)));
        });
        yield return ("Generic.OffsetFaceWithInnerWire", c => c.Sub(new Rectangle(1, 1), new RegularPolygon(0.25, 3)));
        yield return ("Generic.OffsetFaceWithMinLengthAndInner", c => c.Sub(new Circle(0.5), new RegularPolygon(0.25, 3)));
        yield return ("Generic.SplitFace", c => c.Split(new Rectangle(10, 10).SingleFace(), Plane.YZ));
        yield return ("Generic.SplitEdge", c => c.Split(Edge.MakeLine((-5, 0), (5, 0)), Plane.YZ));
        yield return ("Generic.SplitSolid", c => c.Split(new Box(10, 10, 10).SingleSolid(), Plane.YZ));
        yield return ("Generic.SplitBox", c => c.Split(new Box(10, 10, 10), Face.MakeRect(20, 20), Keep.Bottom));
        yield return ("Generic.SweepPathFromEdges", c =>
        {
            double d = 8.5, h = 4.65, lip = 0.5;
            var sk = Fillet(new Rectangle(d * 3, d).Vertices().GroupBy(Axis.X).First, d / 2);
            var p = c.Extrude(sk, 4.65);
            var topEdges = p.Edges().GroupBy(Axis.Z)[2].SortBy(Axis.X)[0..3].SortBy(Axis.Y);
            var sk2 = Plane.ZY.Offset(-d * 3 / 2) * (new Pos(h, -d / 2) * new Rectangle(2 * lip, 2 * lip));
            c.Sub(p, Sweep(sk2, topEdges));
        });

        // ---- Operations/OperationsPartTests.cs ----
        yield return ("Part.ExtrudeBoth", c => c.Extrude(new Rectangle(5, 5), 2.5, theBoth: true));
        yield return ("Part.ExtrudeUntil", c =>
        {
            B.Shape box = new Box(10, 10, 10, align: (Align.Center, Align.Center, Align.Min));
            var target = c.Sub(box, Scale(box, by: (0.8, 0.8, 0.8)));
            var rib = Operations.Extrude(new Rectangle(1, 1), until: Until.Next, target: target);
            c.Add(target, rib);
        });
        yield return ("Part.ExtrudeUntil2", c => c.Sub(new Box(10, 5, 5), new Pos(2.5) * new Cylinder(0.5, 5)));
        yield return ("Part.ExtrudeUntil3", c =>
        {
            var sketch = Plane.XZ * c.Sub(new Rectangle(8, 8, align: Align.Min), new Pos(1, 1) * new Rectangle(7, 7, align: Align.Min));
            var p = c.Extrude(sketch, 2, theBoth: true);
            var profile = new Plane((-2, 0, -2), (0, 1, 0), (1, 0, 1));
            var rib = Operations.Extrude(profile * new Rectangle(4, 1), until: Until.Next, target: p);
            c.Add(p, rib);
        });
        yield return ("Part.ExtrudeUntilFromAFaceOfTheTarget", c =>
        {
            var section = c.Sub(new Rectangle(20, 10, align: (Align.Center, Align.Min)), new Pos(0, 1) * new Circle(6, align: (Align.Center, Align.Min)));
            c.Extrude(Plane.YZ.Offset(-10) * section, 20);
            c.Add(new RectangleRounded(10, 3, 1, rotation: 0), new RectangleRounded(10, 3, 1, rotation: 90));
        });
        yield return ("Part.ExtrudeNonPlanarFace", c => c.Split(new Cylinder(1, 2), Plane.XZ));
        yield return ("Part.LoftReturnIsCleaned", c => c.Loft(
        [
            new Face(Wire.MakePolygon([(0, 0), (1, 0), (2, 0), (2, 1), (0, 1)])),
            new Pos(0, 0, 1) * Face.MakeRect(2, 1, new Plane((1, 0.5, 0))),
        ]));
        yield return ("Part.SimpleLoft", c =>
        {
            const int sliceCount = 10;
            c.Loft(Enumerable.Range(0, sliceCount + 1).Select(i =>
                (B.Shape)new Face(new Wire([Edge.MakeCircle(10 * Math.Sin(i * Math.PI / sliceCount) + 5).Moved(new B.Location((0, 0, i * 3)))]))));
        });
        yield return ("Part.LoftVertex", c => c.Loft([new Rectangle(1, 1), new Vertex(0, 0, 3)], theRuled: true));
        yield return ("Part.LoftWithHole", c =>
        {
            var lower = c.Sub(Face.MakeRect(10, 10), Face.MakeRect(8, 8));
            c.Loft([lower, new Pos(0, 0, 5) * lower]);
        });
        yield return ("Part.LoftWithTwoHoles", c =>
        {
            var lower = c.Sub(c.Sub(new Rectangle(10, 10), new Pos(-2.5) * new Circle(1)), new Pos(2.5) * new Circle(1.5));
            var upper = new Pos(0, 0, 5) * c.Sub(c.Sub(new Rectangle(10, 10), new Pos(2.5) * new Circle(1.5)), new Pos(-2.5) * new Circle(1));
            c.Loft([lower, upper]);
        });
        yield return ("Part.RevolveReturnIsCleaned", c =>
        {
            var profile = new Pos(5, 0, 0) * new Face(Wire.MakePolygon([(0, 0), (1, 0), (2, 0), (2, 1), (0, 1)]));
            c.Revolve(profile, Axis.Y);
        });
        yield return ("Part.SectionCircle", c =>
        {
            c.Section(new Sphere(10), [Plane.XZ]);
            c.Section(new Sphere(10), [Plane.XY], 6);
        });
        yield return ("Part.SectionManyPlanes", c => c.Section(new Sphere(10), [Plane.XY, Plane.YZ]));
        yield return ("Part.SectionMovedObject", c => c.Section(new Pos(-100, 100) * new Sphere(10), [Plane.XY]));
        yield return ("Part.SplitSphere", c =>
        {
            c.Split(new Sphere(10), Plane.XZ);
            c.Split(new Sphere(10), Plane.YZ);
        });
        yield return ("Part.SplitWrappedObject", c =>
        {
            var obj = new Box(1, 1, 1);
            c.Split(Fillet(obj.Edges().GroupBy(Axis.Z).Last, 0.1), Plane.YZ);
        });
        yield return ("Part.ThickenFaces", c =>
        {
            c.Thicken(new RectangleRounded(10, 10, 1), 1);
            c.Thicken(new Sphere(1).Faces()[0], 0.1);
        });
        yield return ("Part.ThickenBoth", c => c.Thicken(Face.MakeRect(2, 2), 1, theBoth: true));

        // ---- Operations/OperationsSketchTests.cs ----
        yield return ("Sketch.FilletedFaceIsUpsideDown", c =>
        {
            var f1 = new Face(Wire.MakePolygon([(1, 0), (1.5, 0.5), (1, 2), (3, 1), (2, 0), (1, 0)]));
            c.Sub(c.Sub(Fillet(f1.Vertices()[2..4], 0.2), new Pos(1.8, 1.2, 0) * new Rectangle(0.2, 0.4)), new Pos(2, 0.5, 0) * new Circle(0.2));
        });
        yield return ("Sketch.MakeHullFlipped", c => c.Add(new Circle(55.0 / 2), new Pos(0, 125) * new Circle(30.0 / 2)));
        yield return ("Sketch.MakeHullOfEdges", c => c.Add(new Pos(-10, 0) * new Circle(10), new Pos(10, 0) * new Circle(7)));
        yield return ("Sketch.TraceLines", c =>
        {
            c.Trace(Edge.MakeLine((0, 0), (10, 0)), 1);
            c.Trace(Wire.MakePolygon([(0, 0), (10, 10), (20, 10)], close: false), 4);
        });

        // ---- Topology/SkipCleanTests.cs, CleanTests.cs ----
        yield return ("SkipClean.BooleanOpLeavesSplitterFaces", c =>
        {
            var left = Solid.MakeBox(1, 1, 1);
            var right = Solid.MakeBox(1, 1, 1, new Plane((1, 0, 0)));
            c.Fuse(left, [right]);
            using (new SkipClean())
            {
                c.Fuse(left, [right]);
            }
        });

        // ---- Topology/HistoryTests.cs ----
        yield return ("History.BooleanRecordsUntouchedModifiedRemovedAndGenerated", c =>
        {
            var box = Solid.MakeBox(10, 10, 10);
            c.Fuse(box, [Solid.MakeCylinder(2, 5, new Plane((5, 5, 10)))]);
            c.Fuse(box, [Solid.MakeCylinder(2, 20, new Plane((5, 5, -5)))]);
        });
        yield return ("History.CleanHistoryIsMergedIntoTheBoolean", c =>
            c.Fuse(Solid.MakeBox(10, 10, 10), [Solid.MakeBox(10, 10, 2, new Plane((10, 0, 0)))]));
        yield return ("History.RecordedPlacements", c => c.Sub(CBox(10, 10, 10), CCylinder(2, 20)));
        yield return ("History.RelocationHistoryPreservesRelativeLocations", c =>
        {
            var box = Solid.MakeBox(10, 10, 10);
            c.Add(box.Moved(new Pos(20, 0, 0)), box.Moved(new Pos(-20, 0, 0)));
        });
        yield return ("History.FusedSketch", c => c.Add(CRectangle(8, 2), CRectangle(2, 8)));
        yield return ("History.FusedPart", c => c.Add(CBox(10, 10, 10), new Pos(z: 5) * CCylinder(2, 5, onBase: true)));
        yield return ("History.CutAndIntersectedParts", c =>
        {
            c.Sub(CBox(10, 10, 10), CCylinder(1, 20));
            c.And(CBox(10, 10, 10), CBox(30, 2, 2));
        });
        yield return ("History.ChainedOperationsReportTheLastOne", c =>
            c.Add(c.Add(CBox(10, 10, 10), new Pos(z: 5) * CCylinder(1, 5, onBase: true)), new Pos(x: 5) * CSphere(1)));
        yield return ("History.EverythingOfSelfWasThereBefore", c =>
        {
            var two = c.Add(CRectangle(2, 2), new Pos(x: 5) * CRectangle(2, 2));
            c.Add(two, new Pos(x: 5) * CRectangle(1, 4));
        });

        // ---- Topology/ShapeTests.cs, CompoundTests.cs ----
        yield return ("Shape.Fuse", c =>
        {
            var box1 = Solid.MakeBox(1, 1, 1);
            var box2 = Solid.MakeBox(1, 1, 1, new Plane((1, 0, 0)));
            c.Fuse(box1, [box2], glue: true);
            c.Fuse(box1, [box2], tolerance: 1e-6);
        });
        yield return ("Compound.Fuse", c =>
        {
            var box1 = Solid.MakeBox(1, 1, 1);
            var box2 = Solid.MakeBox(1, 1, 1, new Plane((1, 0, 0)));
            c.Fuse(new Compound([box1]), [box2], glue: true);
            c.Fuse(new Compound([box1]), [box2], tolerance: 1e-6);
        });
        yield return ("Shape.Split", c =>
        {
            var shape = c.Sub(SBox(1, 1, 1), new Pos(0, 0, -0.25) * SBox(1, 0.5, 0.5));
            c.Split(shape, Plane.XY, Keep.Bottom);
            c.Split(shape, Plane.XY, Keep.Top);
            var s = Solid.MakeCone(1, 0.5, 2, Plane.YZ.Offset(10));
            c.Split(s, Solid.MakeSphere(11).Rotate(Axis.Z, 90).SingleFace());
        });
        yield return ("Shape.SplitByShell", c =>
            c.Split(Solid.MakeBox(5, 5, 1), Shell.Extrude(Wire.MakeRect(4, 4), new Vector(0, 0, 1))));
        yield return ("Shape.SplitKeepAll", c => c.Split(SBox(1, 1, 1), Plane.XY, Keep.All));
        yield return ("Shape.SplitEdgeByShell", c =>
            c.Split(Edge.MakeLine((-5, 0, 0), (5, 0, 0)), Shell.Extrude(Wire.MakeRect(4, 4), new Vector(0, 0, 1))));
        yield return ("Shape.CleanError", c =>
        {
            var sphere = Solid.MakeSphere(1);
            var divider = Solid.MakeBox(0.1, 3, 3, new Plane((-0.05, -1.5, -1.5)));
            foreach (var s in c.Cut(sphere, divider).Solids())
            {
                c.Clean(s);
            }
        });
        yield return ("Shape.IsManifold", c => c.Cut(Solid.MakeBox(1, 1, 1).SingleShell(), Solid.MakeBox(0.5, 0.5, 0.5)));
        yield return ("Shape.IntersectYieldingDisjointPieces", c =>
        {
            var blocks = c.Add(new Pos(-6, 0, 0) * SBox(2, 2, 2), new Pos(6, 0, 0) * SBox(2, 2, 2));
            c.And(SBox(20, 1, 1), blocks);
        });

        // ---- Topology/Shape3DTests.cs, SolidTests.cs ----
        yield return ("Shape3D.PlainMessage", c => c.Sub(PBox(20, 20, 12), new Pos(7, 7, 4) * PBox(12, 12, 12)));
        yield return ("Solid.ExtrudeTaperDraftFallsBackToLoft", c => c.Add(new Rectangle(5.3, 5.3 / 6), new Rectangle(5.3 / 6, 5.3)));
        yield return ("Solid.ExtrudeTaperStar", c =>
        {
            var bar = Face.MakeRect(10, 1);
            c.Add(c.Add(bar, bar.Rotate(Axis.Z, 60)), bar.Rotate(Axis.Z, 120));
        });
        yield return ("Solid.ExtrudeTaperCollapsingProfile", c => c.Add(Face.MakeRect(10, 1), Face.MakeRect(1, 10)));
        yield return ("Solid.HollowSweep", c => c.Sub(Face.MakeRect(1, 1), Face.MakeRect(0.1, 0.1)));

        // ---- Topology/IntersectionTests.cs: its solids and faces, and its 2D and 3D matrices' shape pairs ----
        foreach ((string, Action<Build123dBooleanCapture>) aCase in IntersectionScenarios())
        {
            yield return aCase;
        }
    }

    private static IEnumerable<(string, Action<Build123dBooleanCapture>)> IntersectionScenarios()
    {
        Edge Line(Vector a, Vector b) => Edge.MakeLine(a, b);
        Edge CenterArc(Vector center, double radius, double startAngle, double arcSize)
        {
            double aNormalized = (arcSize + 360.0) % 360.0;
            double anEnd = startAngle + aNormalized;
            if (aNormalized == 360.0)
            {
                startAngle = anEnd;
            }
            return Edge.MakeCircle(radius, new Plane(center), startAngle, anEnd);
        }

        Edge RadiusArc(Vector start, Vector end, double radius)
        {
            double aLength = (end - start).Length / 2.0;
            double aSagitta = Math.Abs(radius) - Math.Sqrt(radius * radius - aLength * aLength);
            var aMid = (end + start) * 0.5;
            var aSagittaVector = ((end - start).Normalized() * Math.Abs(aSagitta)).Rotate(Axis.Z, aSagitta > 0 ? 90 : -90);
            return Edge.MakeThreePointArc(start, aMid + aSagittaVector, end);
        }

        var ed1 = Line((0, 0), (5, 0));
        var ed2 = Line((0, -1), (5, 1));
        var ed3 = Line((0, 0, 5), (5, 0, 5));
        var ed4 = CenterArc((3, 1), 2, 0, 360);
        var ed5 = CenterArc((3, 1), 5, 0, 360);
        var ed6 = Edge.MakeLine((0, -1), (2, 1));
        var ed7 = Edge.MakeLine((0, 1), (2, -1));
        var ed8 = Edge.MakeLine((0, 0), (2, 0));
        var wi1 = (Wire)(new Wire() + new B.Shape[] { Line((0, 0), (1, 0)), RadiusArc((1, 0), (3, 1.5), 2) });
        var wi2 = (Wire)(wi1 + Line((3, 1.5), (3, -1)));
        var wi3 = (Wire)(new Wire() + new B.Shape[] { Line((0, 0), (1, 0)), RadiusArc((1, 0), (3, 0), 2), Line((3, 0), (5, 0)) });
        var wi4 = (Wire)(new Wire() + new B.Shape[] { Line((0, 1), (2, -1)), Line((2, -1), (3, -1)) });
        var wi5 = (Wire)(wi4 + Line((3, -1), (4, 1)));
        var wi6 = (Wire)(new Wire() + new B.Shape[] { Line((0, 1, 1), (2, -1, 1)), Line((2, -1, 1), (4, 1, 1)) });

        var fc1 = FRect(5, 5);
        var fc2 = new Pos(z: 5) * FRect(5, 5);
        var fc3 = new Rotation(y: 90) * FRect(5, 5);
        var fc4 = new Rotation(z: 45) * FRect(5, 5);
        var fc5 = new Pos(2.5, 2.5, 2.5) * new Rotation(0, 90) * FRect(5, 5);
        var fc6 = new Pos(2.5, 2.5) * new Rotation(0, 90, 45, Extrinsic.XYZ) * FRect(5, 5);
        var fc7 = (new Rotation(90) * SCylinder(2, 4)).Faces().FilterBy(GeomType.Cylinder)[0];
        var fc8 = new Face(Wire.MakePolygon([(-1.5, 1, 1), (-1.5, -1, 1), (3.5, -1, -1), (3.5, 1, -1), (-1.5, 1, 1)], close: false));
        var fc11 = FRect(4, 4);
        var fc22 = Shell.Sweep(new Rotation(90) * CenterArc((0, 0), 2, 0, 180), new Wire(Line((0, 2), (0, -2)))).Clean();
        var sh1 = new Shell([new Pos(-4) * fc11, .. fc22.Faces()]);
        var sh3 = new Shell([new Pos(-4) * fc11, .. fc22.Faces(), new Pos(2, 0, -2) * new Rotation(0, 90) * fc11]);
        var sh4 = new Shell([new Pos(-4) * fc11, .. fc22.Faces(), new Pos(4) * fc11]);
        var sh5 = new Pos(z: 1) * new Shell([new Pos(-2, 0, -2) * new Rotation(0, -90) * fc11, .. fc22.Faces(), new Pos(2, 0, -2) * new Rotation(0, 90) * fc11]);
        var sh6 = SBox(2, 2, 2).SingleShell();
        var fc10 = new Pos(1, 0, 0) * (new Rotation(0, 90, 0) * FRect(2, 2));

        // 2D: faces and shells against edges, wires, faces and shells
        (string, B.Shape, object[])[] a2d =
        [
            ("Fc1 Ed1", fc1, [ed1]), ("Fc3 Ed1", new Pos(1) * fc3, [ed1]), ("Fc3 Ed4", new Pos(1.1) * fc3, [ed4]),
            ("Fc1 Wi6", fc1, [wi6]), ("Fc3 Wi4", new Pos(1) * fc3, [wi4]), ("Fc4 Wi5", new Rotation(90) * fc4, [wi5]),
            ("Fc1 Fc2", fc1, [fc2]), ("Fc1 Fc3", fc1, [fc3]), ("Fc1 Fc4", fc1, [fc4]), ("Fc1 Fc5", fc1, [fc5]), ("Fc1 Fc6", fc1, [fc6]),
            ("Fc1 Fc7", fc1, [fc7]), ("Fc7 Fc7", fc7, [new Pos(y: 2) * fc7]), ("Sh2 Fc1", new Pos(z: 1) * sh1, [fc1]),
            ("Sh3 Fc1", new Pos(z: 1) * sh3, [fc1]), ("Sh1 Fc1", sh1, [fc1]), ("Sh4 Fc1", sh4, [fc1]), ("Sh5 Fc1", sh5, [fc1]),
            ("Sh6 Fc1", sh6, [new Pos(0, 0, 1) * fc1]), ("Sh6 Sh6", sh6, [new Pos(2, 1, 1) * sh6]),
            ("Fc1 Fc4 Fc1", fc1, [fc4, new Pos(2, 2) * fc1]), ("Fc1 Ed1 Fc1", fc1, [ed1, new Pos(2.5, 2.5) * fc1]),
            ("Fc7 Wi5 Fc1", fc7, [wi5, fc1]), ("Fc1 Fc8", fc1, [fc8]), ("Fc10 Fc1", fc10, [fc1]),
            ("Fc3 Ax1", fc3, [Axis.X]), ("Fc1 Ax1", fc1, [Axis.X]), ("Fc1 Pl3", fc1, [Plane.XY.Offset(5)]), ("Fc1 Pl1", fc1, [Plane.YZ]),
            ("Fc1 Pl2", fc1, [Plane.XY]), ("Fc7 Pl1", fc7, [Plane.YZ]),
        ];
        // 1D: edges and wires against axes, planes, edges and wires
        (string, B.Shape, object[])[] a1d =
        [
            ("Ed3 Ax1", ed3, [Axis.X]), ("Ed2 Ax1", ed2, [Axis.X]), ("Ed1 Ax1", ed1, [Axis.X]), ("Ed4 Ax1", ed4, [Axis.X]),
            ("Ed1 Pl3", ed1, [Plane.XY.Offset(5)]), ("Ed1 Pl1", ed1, [Plane.YZ]), ("Ed1 Pl2", ed1, [Plane.XY]), ("Ed5 Pl1", ed5, [Plane.YZ]),
            ("Ed3 Ed1", ed3, [ed1]), ("Ed2 Ed1", ed2, [ed1]), ("Ed1 Ed1", ed1, [ed1]), ("Ed4 Ed1", ed4, [ed1]),
            ("Ed6 Ed7 Ed8", ed6, [ed7, ed8]), ("Ed6 Ed7 Pl5", ed6, [ed7, Plane.YZ.Offset(1)]),
            ("Wi6 Ax1", wi6, [Axis.X]), ("Wi4 Ax1", wi4, [Axis.X]), ("Wi1 Ax1", wi1, [Axis.X]), ("Wi5 Ax1", wi5, [Axis.X]),
            ("Wi2 Ax1", wi2, [Axis.X]), ("Wi3 Ax1", wi3, [Axis.X]), ("Wi6 Ed1", wi6, [ed1]), ("Wi4 Ed1", wi4, [ed1]),
            ("Wi1 Ed1", wi1, [ed1]), ("Wi5 Ed1", wi5, [ed1]), ("Wi2 Ed1", wi2, [ed1]), ("Wi3 Ed1", wi3, [ed1]),
            ("Fc4 Wi2", new Rotation(90) * fc4, [wi2]), ("Fc1 Wi1", fc1, [wi1]),
        ];
        foreach ((string aName, B.Shape anObj, object[] aTargets) in a1d)
        {
            yield return ($"Intersection.Shape1d {aName}", c => c.Intersect(anObj, aTargets));
        }

        foreach ((string aName, B.Shape anObj, object[] aTargets) in a2d)
        {
            yield return ($"Intersection.Shape2d {aName}", c => c.Intersect(anObj, aTargets));
        }

        // 3D
        var sl1 = SBox(2, 2, 2);
        var sl2 = new Pos(z: 5) * SBox(2, 2, 2);
        Solid? sl3 = null;
        B.Part? sl5 = null, sl9 = null;
        var sl6 = new Pos(2, 0, 1.5) * SBox(2, 2, 1);
        var sl7 = new Pos(0, 0.1, 0) * SBox(2, 2, 2);
        var sl8 = new Pos(1, 0, -1) * SBox(4, 2, 1);
        yield return ("Intersection.Sl3", c => sl3 = (Solid)c.Sub(SCylinder(2, 1), SCylinder(1.5, 1)));
        yield return ("Intersection.Sl5", c => sl5 = new B.Part([c.Add(new Pos(0.5, 0, 1) * SBox(1, 1, 1), new Pos(0.5, 0, 1) * SBox(2, 0.1, 1))]));
        yield return ("Intersection.Sl9", c => sl9 = new B.Part([c.Add(SBox(2, 2, 2), sl5!)]));
        yield return ("Intersection.HalfSphereSolid", c => c.And(Solid.MakeSphere(1), new Pos(0, 0, 0.5) * SBox(3, 3, 2)));
        Func<(string, B.Shape, object[])[]> a3d = () =>
        [
            ("Sl1 Ed1", sl1, [ed1]), ("Sl1 Ed1 moved", sl1, [new Pos(0, 1, 1) * ed1]), ("Sl1 Ed4", new Pos(2.1, 1) * sl1, [ed4]),
            ("Sl1 Ed4 boundary", new Pos(2.1, 1, -1) * sl1, [ed4]), ("Sl1 Wi6", new Pos(2, 0.5, 1) * sl1, [wi6]),
            ("Sl1 Wi6 boundary", new Pos(2, 0.5, 2) * sl1, [wi6]), ("Sl2 Fc1", sl2, [fc1]), ("Sl1 Fc1", sl1, [fc1]),
            ("Sl1 Fc1 below", new Pos(0, 0, -1) * sl1, [fc1]), ("Sl1 Fc1 above", new Pos(0, 0, 1) * sl1, [fc1]),
            ("Sl1 Fc1 edge", new Pos(3.5, 0, 1) * sl1, [fc1]), ("Sl1 Fc7", new Pos(0.9) * sl1, [fc7]), ("Sl1 Fc7 b", new Pos(0.9, 1) * sl1, [fc7]),
            ("Sl1 Fc7 c", new Pos(0.9, 1.5) * sl1, [fc7]), ("Sl2 Sh1", sl2, [sh1]), ("Sl1 Sh1", new Pos(-2) * sl1, [sh1]),
            ("Sl1 Sh1 side", new Pos(-2, 3) * sl1, [sh1]), ("Sl1 Sl2", sl1, [sl2]), ("Sl1 Sl1", sl1, [new Pos(1, 1, 1) * sl1]),
            ("Sl1 Sl1 edge", sl1, [new Pos(2, 2, 1) * sl1]), ("Sl1 Sl1 face", sl1, [new Pos(2, 1.5, 1) * sl1]),
            ("Sl1 Sl1 corner", sl1, [new Pos(2, 2, 2) * sl1]), ("Sl1 Sl3", sl1, [new Pos(0.45) * sl3!]),
            ("Sl1 Sl1 touch", sl1, [new Pos(2, 0, 0) * sl1]), ("Sl1 Sl3 Sl1", new Pos(1.5, 1.5) * sl1, [sl3!, new Pos(0.5, 0.5) * sl1]),
            ("Sl1 Sl3 Sl1 b", new Pos(0, 1.5) * sl1, [sl3!, new Pos(0.5, 0.5) * sl1]),
            ("Sl1 Sl3 Sl1 c", new Pos(0.5, 1.5) * sl1, [sl3!, new Pos(0.5, 0.5) * sl1]),
            ("Sl1 Sl3 Fc1", new Pos(1.5, 1.5) * sl1, [sl3!, new Pos(z: 0.5) * fc1]), ("Sl5 Sl6", sl5!, [sl6]), ("Sl7 Sl8", sl7, [sl8]),
            ("Sl9 Sl6", sl9!, [sl6]), ("Sl1 Ax1", sl1, [Axis.X]), ("Sl1 Pl3", sl1, [Plane.XY.Offset(5)]), ("Sl1 Pl2", sl1, [Plane.XY]),
            ("Sl1 Pl2 offset", sl1, [Plane.XY.Offset(1)]),
        ];
        yield return ("Intersection.Shape3d", c =>
        {
            foreach ((string aName, B.Shape anObj, object[] aTargets) in a3d())
            {
                c.myScenario = $"Intersection.Shape3d {aName}";
                c.myIndex = 0;
                c.Intersect(anObj, aTargets);
            }
        });

        // compounds
        yield return ("Intersection.Compounds", c =>
        {
            B.Shape[] Grid2(double spacing, B.Shape shape) => [new Pos(-spacing / 2) * shape, new Pos(spacing / 2) * shape];
            var cp3 = new Compound(Grid2(5, FRect(2, 2)));
            var cp4 = new Compound(Grid2(5, SBox(2, 2, 2)));
            c.Intersect(cp3, [fc1]);
            c.Intersect(cp4, [fc1]);
            c.Intersect(cp4, [new Pos(z: 1) * cp4]);
            c.myScenario = "Intersection.Sketches";
            c.Add(new Sketch(), [fc1, fc2, fc3]);
            c.Add(new Part(), [sl1, sl2, sl3!]);
            c.Add(new Sketch(), Grid2(5, FCircle(2)));
            c.myScenario = "Intersection.Steps";
            c.Add(c.Add(c.Sub(SBoxMin(2, 2, 0.5), SBoxMin(1, 1.2, 1)), new Pos(1, 0, 0) * SBoxMin(1, 1, 1)), SBoxMin(1, 2, 0.5));
        });
    }
}
#endif
