// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Classifier.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Classifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Classifies points against the wires of a face in its parametric space: a point is inside if it is inside every
/// outer wire and outside every hole.
/// </summary>
/// <remarks>A working object, as OCCT's: <see cref="RegisterWire"/> adds a wire.</remarks>
public sealed class BRepMesh_Classifier : Transient
{
    private readonly NCollection_DynamicArray<Class2d> myTabClass = new();
    private readonly NCollection_DynamicArray<bool> myTabOrient = new();

    /// <summary>A classifier without wires.</summary>
    public BRepMesh_Classifier() { }

    /// <summary>Whether the point is inside the face (on a wire counts as outside).</summary>
    public Containment Perform(in Point2D thePoint)
    {
        bool isOut = false;
        int aNb = myTabClass.Length();

        for (int i = 0; i < aNb; i++)
        {
            int aCur = (int)myTabClass[i].SiDans(thePoint);
            if (aCur == 0)
            {
                // Point is ON, but mark it as OUT
                isOut = true;
            }
            else
            {
                isOut = myTabOrient[i] ? (aCur == -1) : (aCur == 1);
            }

            if (isOut)
            {
                return Out;
            }
        }

        return In;
    }

    /// <summary>
    /// Registers a wire given by its points (OCCT passes pointers to them), with the tolerances in U and V and the
    /// face's ranges in U and V. Its orientation (outer or hole) is taken from the sign of its accumulated turning
    /// angle.
    /// </summary>
    public void RegisterWire(NCollection_Sequence<Point2D> theWire,
                             (double first, double second) theTolUV,
                             (double first, double second) theRangeU,
                             (double first, double second) theRangeV)
    {
        int aNbPnts = theWire.Length();
        if (aNbPnts < 2)
        {
            return;
        }

        // Accumulate angle
        NCollection_Array1<Point2D> aPClass = new(1, aNbPnts);
        double anAngle = 0.0;
        Point2D p1 = theWire[1], p2 = theWire[2], p3;
        aPClass[1] = p1;
        aPClass[2] = p2;

        double aAngTol = global::Kerf.Engine.Foundation.Precision.Angular();
        double aSqConfusion = global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion();

        for (int i = 1; i <= aNbPnts; i++)
        {
            int ii = i + 2;
            if (ii > aNbPnts)
            {
                p3 = aPClass[ii - aNbPnts];
            }
            else
            {
                p3 = theWire.Value(ii);
                aPClass[ii] = p3;
            }

            Vector2D A = new(p1, p2), B = new(p2, p3);
            if (A.SquareMagnitude() > aSqConfusion && B.SquareMagnitude() > aSqConfusion)
            {
                double aCurAngle = A.Angle(B);
                double aCurAngleAbs = Math.Abs(aCurAngle);
                // Check if vectors are opposite
                if (aCurAngleAbs > aAngTol && (M_PI - aCurAngleAbs) > aAngTol)
                {
                    anAngle += aCurAngle;
                    p1 = p2;
                }
            }

            p2 = p3;
        }

        // Check for zero angle - treat self intersecting wire as outer
        if (Math.Abs(anAngle) < aAngTol)
        {
            anAngle = 0.0;
        }

        myTabClass.Append(new Class2d(aPClass,
                                            theTolUV.first,
                                            theTolUV.second,
                                            theRangeU.first,
                                            theRangeV.first,
                                            theRangeU.second,
                                            theRangeV.second));

        myTabOrient.Append(!(anAngle < 0.0));
    }
}
