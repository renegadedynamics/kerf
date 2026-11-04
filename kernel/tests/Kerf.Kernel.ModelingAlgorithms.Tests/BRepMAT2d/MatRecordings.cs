// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Offsets.Bisectors;
using Kerf.Engine.Offsets.MedialAxis;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;

using static Kerf.Engine.Offsets.MedialAxis.MedialSide;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Native OCCT's bisecting loci of planar faces recorded by tools/matrec (unit 5M): BRepMAT2d_Explorer,
/// BRepMAT2d_BisectingLocus::Compute and BRepMAT2d_LinkTopoBilo on polygons, arcs, holed regions, circles and open
/// wires, on both sides, with both join types. Each line holds a case's inputs as text (both sides build the face from
/// it) and native's results as named tokens, every double as its 64 bits in hex. <see cref="Run"/> replays a case on
/// the port and writes the same tokens, mirroring tools/matrec/main.cpp.
/// </summary>
internal static class MatRecordings
{
    public sealed record Case(string Input, string[] Native);

    private static readonly Lazy<List<Case>> theCases = new(Load);

    public static List<Case> Cases => theCases.Value;

    private static List<Case> Load()
    {
        using Stream aRes = typeof(MatRecordings).Assembly.GetManifestResourceStream("BRepMAT2d.matrec.txt.gz")
                            ?? throw new InvalidOperationException("BRepMAT2d.matrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        List<Case> aCases = [];
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            if (aLine.Length == 0)
            {
                continue;
            }
            int aSemi = aLine.IndexOf(';');
            string[] aOut = aLine[(aSemi + 1)..].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            aCases.Add(new Case(aLine[..aSemi].Trim(), aOut));
        }
        return aCases;
    }

    public static string Hex(double theValue) =>
        BitConverter.DoubleToInt64Bits(theValue).ToString("x", CultureInfo.InvariantCulture);

    private static string Hex(Point2D theP) => Hex(theP.X) + "," + Hex(theP.Y);

    private static string ExcName(Exception theE) => Refusal.IsFailure(theE) ? "!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(theE.GetType()) : "!SEH";

    private static string Fail(Action theF)
    {
        try
        {
            theF();
        }
        catch (Exception anE)
        {
            return ExcName(anE);
        }
        return "";
    }

    //=============================================================================================
    // The face, as matrec's ReadWire and ReadFace

    private sealed class Reader(string theText)
    {
        private readonly string[] myWords = theText.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        private int myPos;

        public string Word() => myWords[myPos++];

        public double D() => double.Parse(Word(), CultureInfo.InvariantCulture);

        public int I() => int.Parse(Word(), CultureInfo.InvariantCulture);
    }

    private static Wire ReadWire(Reader theR, out bool theClosed)
    {
        string aKind = theR.Word();
        if (aKind == "O")
        {
            double cx = theR.D(), cy = theR.D(), r = theR.D();
            Circle3D aC = new(new Frame3D(new Point3D(cx, cy, 0.0), Constants.DZ()), r);
            theClosed = true;
            return new MakeWire(new MakeEdge(aC).Edge()).Wire();
        }
        theClosed = theR.I() != 0;
        int aNb = theR.I();
        List<Point3D> aPnts = [];
        List<string> aKinds = [];
        List<Point3D> aMids = [];
        List<Point3D> aMids2 = [];
        List<Vertex> aVerts = [];
        for (int i = 0; i < aNb; i++)
        {
            double x = theR.D(), y = theR.D();
            aPnts.Add(new Point3D(x, y, 0.0));
            string k = theR.Word();
            aKinds.Add(k);
            if (k == "A")
            {
                double mx = theR.D(), my = theR.D();
                aMids.Add(new Point3D(mx, my, 0.0));
                aMids2.Add(new Point3D());
            }
            else if (k == "S")
            {
                double mx = theR.D(), my = theR.D(), nx = theR.D(), ny = theR.D();
                aMids.Add(new Point3D(mx, my, 0.0));
                aMids2.Add(new Point3D(nx, ny, 0.0));
            }
            else
            {
                aMids.Add(new Point3D());
                aMids2.Add(new Point3D());
            }
            aVerts.Add(new MakeVertex(aPnts[^1]).Vertex());
        }
        MakeWire aMW = new();
        int aNbEdges = theClosed ? aNb : aNb - 1;
        for (int i = 0; i < aNbEdges; i++)
        {
            int j = (i + 1) % aNb;
            if (aKinds[i] == "A")
            {
                TrimmedCurve3D anArc = new MakeArcOfCircle(aPnts[i], aMids[i], aPnts[j]).Value();
                aMW.Add(new MakeEdge(anArc, aVerts[i], aVerts[j]).Edge());
            }
            else if (aKinds[i] == "S")
            {
                NCollection_Array1<Point3D> aPoles = new(1, 4);
                aPoles[1] = aPnts[i];
                aPoles[2] = aMids[i];
                aPoles[3] = aMids2[i];
                aPoles[4] = aPnts[j];
                BezierCurve3D aB = new(aPoles);
                aMW.Add(new MakeEdge(aB, aVerts[i], aVerts[j]).Edge());
            }
            else
            {
                aMW.Add(new MakeEdge(aVerts[i], aVerts[j]).Edge());
            }
        }
        return aMW.Wire();
    }

    private static Face ReadFace(Reader theR)
    {
        int aNbWires = theR.I();
        Face aFace = new();
        Plane3D aPln = new(new CoordinateSystem(Constants.XOY()));
        for (int i = 0; i < aNbWires; i++)
        {
            Wire aW = ReadWire(theR, out bool isClosed);
            if (i == 0)
            {
                if (isClosed)
                {
                    aFace = new MakeFace(aPln, aW, true).Face();
                }
                else
                {
                    aFace = new MakeFace(new MakeFace(aPln).Face(), aW).Face();
                }
            }
            else
            {
                aFace = new MakeFace(aFace, aW).Face();
            }
        }
        return aFace;
    }

    /// <summary>The face of a case's input.</summary>
    public static Face Face(string theInput)
    {
        Reader aR = new(theInput);
        aR.Word();
        aR.Word();
        aR.I();
        aR.I();
        return ReadFace(aR);
    }

    //=============================================================================================
    // The results, as matrec's RunCase

    private static string ShapeName(in Shape theS, IndexedSet<Shape> theEdges,
                                    IndexedSet<Shape> theVerts)
    {
        const string anOri = "FRIE";
        if (theS.IsNull)
        {
            return "null";
        }
        string aName;
        if (theS.Type == ShapeType.Edge)
        {
            aName = "E" + (theEdges.IndexOf(theS) + 1).ToString(CultureInfo.InvariantCulture);
        }
        else if (theS.Type == ShapeType.Vertex)
        {
            aName = "V" + (theVerts.IndexOf(theS) + 1).ToString(CultureInfo.InvariantCulture);
        }
        else
        {
            aName = "S" + ((int)theS.Type).ToString(CultureInfo.InvariantCulture);
        }
        return aName + anOri[(int)theS.Orientation];
    }

    private static string GeomName(Geometry2D? theG)
    {
        if (theG is null)
        {
            return "null";
        }
        string aName = global::Kerf.Engine.Foundation.OcctTypeNames.Of(theG.GetType());
        if (theG is TrimmedCurve2D aT)
        {
            aName += "/" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(aT.ChangeBasisCurve().GetType()) + "," + Hex(aT.FirstParameter) + ","
                     + Hex(aT.LastParameter);
        }
        if (theG is CartesianPoint2 aP)
        {
            aName += "," + Hex(aP.Pnt2d());
        }
        return aName;
    }

    private static void Bisec(List<string> theOut, string thePre, Bisec theB)
    {
        TrimmedCurve2D? aC = theB.Value();
        if (aC is null)
        {
            theOut.Add(thePre + "null");
            return;
        }
        Curve2D aBasis = aC.ChangeBasisCurve();
        string aKind = global::Kerf.Engine.Foundation.OcctTypeNames.Of(aBasis.GetType());
        if (aBasis is BisecAna anAna)
        {
            Tok(theOut, thePre + "G", () => global::Kerf.Engine.Foundation.OcctTypeNames.Of(anAna.Geom2dCurve().GetType()));
        }
        theOut.Add(thePre + "T:" + aKind);
        double f = aC.FirstParameter, l = aC.LastParameter;
        theOut.Add(thePre + "R:" + Hex(f) + "," + Hex(l));
        double[] aFrac = [0.0, 0.25, 0.5, 0.75, 1.0];
        double[] aStep = [0.0, 0.5, 1.0, 2.0, 5.0];
        for (int i = 0; i < 5; i++)
        {
            double u = (l >= 1.0e50 || f <= -1.0e50) ? (f <= -1.0e50 ? l - aStep[i] : f + aStep[i])
                                                     : f + (l - f) * aFrac[i];
            Tok(theOut, thePre + "P" + I(i), () => Hex(aC.Value(u)));
        }
    }

    private static string I(int theI) => theI.ToString(CultureInfo.InvariantCulture);

    private static void Tok(List<string> theOut, string theName, Func<string> theF)
    {
        string aVal = "";
        string aFail = Fail(() => aVal = theF());
        theOut.Add(theName + ":" + (aFail.Length == 0 ? aVal : aFail));
    }

    /// <summary>Replays a case on the port, writing matrec's tokens.</summary>
    public static List<string> Run(string theInput)
    {
        List<string> anOut = [];
        Reader aR = new(theInput);
        aR.Word();
        string aSide = aR.Word();
        int aJoin = aR.I();
        int anOpen = aR.I();
        Face aFace = new();
        string aFail = Fail(() => aFace = ReadFace(aR));
        if (aFail.Length != 0)
        {
            anOut.Add("face:" + aFail);
            return anOut;
        }
        IndexedSet<Shape> anEdges = new(ShapeHasher.Instance);
        IndexedSet<Shape> aVerts = new(ShapeHasher.Instance);
        Exploration.MapShapes(aFace, ShapeType.Edge, anEdges);
        Exploration.MapShapes(aFace, ShapeType.Vertex, aVerts);
        anOut.Add("nE:" + I(anEdges.Count));

        MedialAxisExplorer anExplo = new();
        aFail = Fail(() => anExplo.Perform(aFace));
        if (aFail.Length != 0)
        {
            anOut.Add("explo:" + aFail);
            return anOut;
        }
        anOut.Add("nC:" + I(anExplo.NumberOfContours()));
        for (int ic = 1; ic <= anExplo.NumberOfContours(); ic++)
        {
            anOut.Add("c" + I(ic) + ":" + I(anExplo.NumberOfCurves(ic)));
        }
        anOut.Add("mod:" + (anExplo.IsModified(aFace) ? "1" : "0"));

        BisectingLocus aLocus = new();
        aFail = Fail(() => aLocus.Compute(anExplo, 1, aSide == "L" ? MedialSide.Left : MedialSide.Right, (JoinKind)aJoin,
                                          anOpen != 0));
        if (aFail.Length != 0)
        {
            anOut.Add("compute:" + aFail);
            return anOut;
        }
        anOut.Add("done:" + (aLocus.IsDone ? "1" : "0"));
        if (!aLocus.IsDone)
        {
            return anOut;
        }
        for (int ic = 1; ic <= aLocus.NumberOfContours(); ic++)
        {
            Tok(anOut, "ne" + I(ic), () => I(aLocus.NumberOfElts(ic)));
            for (int j = 1; j <= anExplo.NumberOfCurves(ic); j++)
            {
                Tok(anOut, "ns" + I(ic) + "." + I(j), () => I(aLocus.NumberOfSections(ic, j)));
            }
            Tok(anOut, "be" + I(ic), () =>
            {
                StringBuilder aS = new();
                for (int ie = 1; ie <= aLocus.NumberOfElts(ic); ie++)
                {
                    aS.Append(ie > 1 ? "," : "").Append(I(aLocus.BasicElt(ic, ie).Index()));
                }
                return aS.ToString();
            });
        }
        MedialGraph aG = aLocus.Graph();
        anOut.Add("g:" + I(aG.NumberOfArcs()) + "," + I(aG.NumberOfNodes()) + "," + I(aG.NumberOfBasicElts()) + ","
                  + I(aG.NumberOfInfiniteNodes()));
        for (int i = 1; i <= aG.NumberOfArcs(); i++)
        {
            string aPre = "a" + I(i);
            Tok(anOut, aPre, () =>
            {
                MedialArc anArc = aG.Arc(i);
                return I(anArc.Index()) + "," + I(anArc.GeomIndex()) + "," + I(anArc.FirstElement().Index()) + ","
                       + I(anArc.SecondElement().Index()) + "," + I(anArc.FirstNode().Index()) + ","
                       + I(anArc.SecondNode().Index());
            });
            aFail = Fail(() =>
            {
                Bisec aB = aLocus.GeomBis(aG.Arc(i), out bool aRev);
                anOut.Add(aPre + "rev:" + (aRev ? "1" : "0"));
                Bisec(anOut, aPre, aB);
            });
            if (aFail.Length != 0)
            {
                anOut.Add(aPre + "bis:" + aFail);
            }
        }
        for (int i = 1; i <= aG.NumberOfNodes(); i++)
        {
            string aPre = "n" + I(i);
            Tok(anOut, aPre, () =>
            {
                MedialNode aN = aG.Node(i);
                return I(aN.Index()) + "," + I(aN.GeomIndex()) + "," + Hex(aN.Distance()) + ","
                       + (aN.Infinite() ? "1" : "0") + (aN.OnBasicElt() ? "1" : "0") + (aN.PendingNode() ? "1" : "0");
            });
            Tok(anOut, aPre + "p", () => Hex(aLocus.GeomElt(aG.Node(i))));
            Tok(anOut, aPre + "l", () =>
            {
                NCollection_Sequence<MedialArc> aS = new();
                aG.Node(i).LinkedArcs(aS);
                StringBuilder aRes = new();
                for (int k = 1; k <= aS.Length(); k++)
                {
                    aRes.Append(k > 1 ? "," : "").Append(I(aS[k].Index()));
                }
                return aRes.ToString();
            });
        }
        for (int i = 1; i <= aG.NumberOfBasicElts(); i++)
        {
            string aPre = "b" + I(i);
            Tok(anOut, aPre, () =>
            {
                MedialBasicElt aBE = aG.BasicElt(i);
                return I(aBE.Index()) + "," + I(aBE.GeomIndex()) + ","
                       + (aBE.StartArc() is null ? "0" : I(aBE.StartArc()!.Index())) + ","
                       + (aBE.EndArc() is null ? "0" : I(aBE.EndArc()!.Index()));
            });
            Tok(anOut, aPre + "g", () => GeomName(aLocus.GeomElt(aG.BasicElt(i))));
        }

        LinkTopoBilo aLink = new();
        aFail = Fail(() => aLink.Perform(anExplo, aLocus));
        if (aFail.Length != 0)
        {
            anOut.Add("link:" + aFail);
            return anOut;
        }
        for (int k = 1; k <= anEdges.Count + aVerts.Count; k++)
        {
            Shape aS = k <= anEdges.Count ? anEdges[k - 1] : aVerts[k - anEdges.Count - 1];
            string aNm = k <= anEdges.Count ? "lE" + I(k) : "lV" + I(k - anEdges.Count);
            Tok(anOut, aNm, () =>
            {
                StringBuilder aRes = new();
                for (aLink.Init(aS); aLink.More(); aLink.Next())
                {
                    aRes.Append(aRes.Length == 0 ? "" : ",").Append(I(aLink.Value().Index()));
                }
                return aRes.ToString();
            });
        }
        for (int i = 1; i <= aG.NumberOfBasicElts(); i++)
        {
            Tok(anOut, "gs" + I(i), () => ShapeName(aLink.GeneratingShape(aG.BasicElt(i)), anEdges, aVerts));
        }
        return anOut;
    }
}
