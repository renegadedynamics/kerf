// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Polygo.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Polygo.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Polygo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Geometry;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// A 2D polygon of points for the interference of a line with the restrictions (IntPatch_RstInt): its segments join
/// consecutive points, and its deflection is its error.
/// </summary>
internal abstract class Polygo : Polygon2d
{
    // OCCT's Dump numbers its polygons with a function-static counter.
    private static int s_dumpNum;

    /// <summary>The deflection of the polygon.</summary>
    protected double _error;

    protected Polygo(double error = 0.0) => _error = error;

    public double Error() => _error;

    public abstract int PointCount { get; }

    public abstract Point2D Point(int index);

    /// <summary>The tolerance of the polygon.</summary>
    public override double DeflectionOverEstimation() => _error;

    /// <summary>The number of segments in the polyline.</summary>
    public override int SegmentCount => PointCount - 1;

    /// <summary>The points of the segment <paramref name="index"/>.</summary>
    public override void Segment(int index, out Point2D begin, out Point2D end)
    {
        begin = Point(index);
        end = Point(index + 1);
    }

    /// <summary>
    /// Writes the polygon as OCCT's <c>Dump</c> prints it, in Draw's <c>pol2d</c>/<c>pnt2d</c> form; the box is
    /// written by its <c>ToString</c> (Bnd_Box2d's <c>Dump</c> is not ported).
    /// </summary>
    public void Dump(TextWriter theOut)
    {
        int num = Interlocked.Increment(ref s_dumpNum);
        theOut.Write("\n#------------- D u m p     B o x 2 d   (" + num.ToString(CultureInfo.InvariantCulture) + ")" + '\n');
        theOut.Write(Bounding().ToString());
        theOut.Write("\n#-----------------------------------------------" + '\n');
        int nbs = SegmentCount;
        theOut.Write("\npol2d " + num.ToString(CultureInfo.InvariantCulture) + " " + nbs.ToString(CultureInfo.InvariantCulture)
                     + " ");
        theOut.Write(R(DeflectionOverEstimation()) + '\n');
        Point2D p = new(), pf = new();
        for (int i = 1; i <= nbs; i++)
        {
            Segment(i, out p, out pf);
            theOut.Write("pnt2d " + num.ToString(CultureInfo.InvariantCulture) + "  " + R(p.X) + " " + R(p.Y) + '\n');
        }
        theOut.Write("pnt2d " + num.ToString(CultureInfo.InvariantCulture) + "  " + R(pf.X) + " " + R(pf.Y) + '\n');
    }

    private static string R(double value) => StreamUtilities.FormatReal(value, 6);
}
