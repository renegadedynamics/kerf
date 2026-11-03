// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Classifier.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Classifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Classifies points against the wires of a face in its parametric space: a point is inside if it is inside every
/// outer wire and outside every hole.
/// </summary>
/// <remarks>A working object, as OCCT's: <see cref="RegisterWire"/> adds a wire.</remarks>
public sealed class Classifier : Transient
{
    private readonly NCollection_DynamicArray<Class2d> _tabClass = new();
    private readonly NCollection_DynamicArray<bool> _tabOrient = new();

    /// <summary>A classifier without wires.</summary>
    public Classifier() { }

    /// <summary>Whether the point is inside the face (on a wire counts as outside).</summary>
    public Containment Perform(in Point2D point)
    {
        bool isOut = false;
        int nb = _tabClass.Length();

        for (int i = 0; i < nb; i++)
        {
            int cur = (int)_tabClass[i].SiDans(point);
            if (cur == 0)
            {
                // Point is ON, but mark it as OUT
                isOut = true;
            }
            else
            {
                isOut = _tabOrient[i] ? (cur == -1) : (cur == 1);
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
    public void RegisterWire(NCollection_Sequence<Point2D> wire,
                             (double first, double second) tolUV,
                             (double first, double second) rangeU,
                             (double first, double second) rangeV)
    {
        int nbPnts = wire.Length();
        if (nbPnts < 2)
        {
            return;
        }

        // Accumulate angle
        NCollection_Array1<Point2D> pClass = new(1, nbPnts);
        double angle = 0.0;
        Point2D p1 = wire[1], p2 = wire[2], p3;
        pClass[1] = p1;
        pClass[2] = p2;

        double angTol = global::Kerf.Engine.Foundation.Precision.Angular();
        double sqConfusion = global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion();

        for (int i = 1; i <= nbPnts; i++)
        {
            int ii = i + 2;
            if (ii > nbPnts)
            {
                p3 = pClass[ii - nbPnts];
            }
            else
            {
                p3 = wire.Value(ii);
                pClass[ii] = p3;
            }

            Vector2D a = new(p1, p2), b = new(p2, p3);
            if (a.SquareMagnitude() > sqConfusion && b.SquareMagnitude() > sqConfusion)
            {
                double curAngle = a.Angle(b);
                double curAngleAbs = Math.Abs(curAngle);
                // Check if vectors are opposite
                if (curAngleAbs > angTol && (M_PI - curAngleAbs) > angTol)
                {
                    angle += curAngle;
                    p1 = p2;
                }
            }

            p2 = p3;
        }

        // Check for zero angle - treat self intersecting wire as outer
        if (Math.Abs(angle) < angTol)
        {
            angle = 0.0;
        }

        _tabClass.Append(new Class2d(pClass,
                                            tolUV.first,
                                            tolUV.second,
                                            rangeU.first,
                                            rangeV.first,
                                            rangeU.second,
                                            rangeV.second));

        _tabOrient.Append(!(angle < 0.0));
    }
}
