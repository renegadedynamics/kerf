// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Polygo.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Polygo.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Polygo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using System.Globalization;

namespace Occt;

/// <summary>
/// A 2D polygon of points for the interference of a line with the restrictions (IntPatch_RstInt): its segments join
/// consecutive points, and its deflection is its error.
/// </summary>
public abstract class IntPatch_Polygo : Intf_Polygon2d
{
    // OCCT's Dump numbers its polygons with a function-static counter.
    private static int s_DumpNum;

    /// <summary>The deflection of the polygon.</summary>
    protected double myError;

    protected IntPatch_Polygo(double theError = 0.0) => myError = theError;

    public double Error() => myError;

    public abstract int NbPoints();

    public abstract Point2D Point(int Index);

    /// <summary>The tolerance of the polygon.</summary>
    public override double DeflectionOverEstimation() => myError;

    /// <summary>The number of segments in the polyline.</summary>
    public override int NbSegments() => NbPoints() - 1;

    /// <summary>The points of the segment <paramref name="theIndex"/>.</summary>
    public override void Segment(int theIndex, out Point2D theBegin, out Point2D theEnd)
    {
        theBegin = Point(theIndex);
        theEnd = Point(theIndex + 1);
    }

    /// <summary>
    /// Writes the polygon as OCCT's <c>Dump</c> prints it, in Draw's <c>pol2d</c>/<c>pnt2d</c> form; the box is
    /// written by its <c>ToString</c> (Bnd_Box2d's <c>Dump</c> is not ported).
    /// </summary>
    public void Dump(TextWriter theOut)
    {
        int num = Interlocked.Increment(ref s_DumpNum);
        theOut.Write("\n#------------- D u m p     B o x 2 d   (" + num.ToString(CultureInfo.InvariantCulture) + ")" + '\n');
        theOut.Write(Bounding().ToString());
        theOut.Write("\n#-----------------------------------------------" + '\n');
        int nbs = NbSegments();
        theOut.Write("\npol2d " + num.ToString(CultureInfo.InvariantCulture) + " " + nbs.ToString(CultureInfo.InvariantCulture)
                     + " ");
        theOut.Write(R(DeflectionOverEstimation()) + '\n');
        Point2D P = new(), PF = new();
        for (int i = 1; i <= nbs; i++)
        {
            Segment(i, out P, out PF);
            theOut.Write("pnt2d " + num.ToString(CultureInfo.InvariantCulture) + "  " + R(P.X()) + " " + R(P.Y()) + '\n');
        }
        theOut.Write("pnt2d " + num.ToString(CultureInfo.InvariantCulture) + "  " + R(PF.X()) + " " + R(PF.Y()) + '\n');
    }

    private static string R(double theValue) => StreamUtilities.FormatReal(theValue, 6);
}
