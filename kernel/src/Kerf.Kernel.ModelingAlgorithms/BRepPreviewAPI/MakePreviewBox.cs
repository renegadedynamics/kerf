// Port of: src/ModelingAlgorithms/TKPrim/BRepPreviewAPI/BRepPreviewAPI_MakeBox.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPreviewAPI/BRepPreviewAPI_MakeBox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Preview;

/// <summary>
/// Builds a valid box if the points fulfil the conditions of a valid box, and a preview otherwise: a vertex if the box
/// is thin in all directions, an edge if it is thin in two, a rectangular face if it is thin in one.
/// </summary>
internal class MakePreviewBox : MakeBox
{
    /// <summary>Constructor; set the box with <c>Init</c>.</summary>
    public MakePreviewBox()
    {
    }

    /// <summary>Creates a preview depending on point values.</summary>
    public override void Build(ProgressRange range = default)
    {
        Point3D location = _wedge.Axes().Location;

        Point3D firstPoint = new(location.X, location.Y, location.Z);
        Point3D secondPoint = new(location.X + _wedge.GetXMax(),
                                  location.Y + _wedge.GetYMax(),
                                  location.Z + _wedge.GetZMax());

        bool thinOnX = Math.Abs(firstPoint.X - secondPoint.X) < global::Kerf.Engine.Foundation.Precision.Confusion();
        bool thinOnY = Math.Abs(firstPoint.Y - secondPoint.Y) < global::Kerf.Engine.Foundation.Precision.Confusion();
        bool thinOnZ = Math.Abs(firstPoint.Z - secondPoint.Z) < global::Kerf.Engine.Foundation.Precision.Confusion();

        int previewType = (thinOnX ? 1 : 0) + (thinOnY ? 1 : 0) + (thinOnZ ? 1 : 0);

        if (previewType == 3) // thin box in all directions is a point
        {
            MakeVertex(firstPoint);
        }
        else if (previewType == 2) // thin box in two directions is a point
        {
            MakeEdge(firstPoint, secondPoint);
        }
        // thin box in only one direction is a rectangular face
        else if (previewType == 1)
        {
            Point3D pnt1 = new(), pnt2 = new(), pnt3 = new(), pnt4 = new();
            if (thinOnX)
            {
                pnt1 = new Point3D(firstPoint.X, firstPoint.Y, firstPoint.Z);
                pnt2 = new Point3D(firstPoint.X, secondPoint.Y, firstPoint.Z);
                pnt3 = new Point3D(firstPoint.X, secondPoint.Y, secondPoint.Z);
                pnt4 = new Point3D(firstPoint.X, firstPoint.Y, secondPoint.Z);
            }
            else if (thinOnY)
            {
                pnt1 = new Point3D(firstPoint.X, firstPoint.Y, firstPoint.Z);
                pnt2 = new Point3D(secondPoint.X, firstPoint.Y, firstPoint.Z);
                pnt3 = new Point3D(secondPoint.X, firstPoint.Y, secondPoint.Z);
                pnt4 = new Point3D(firstPoint.X, firstPoint.Y, secondPoint.Z);
            }
            else if (thinOnZ)
            {
                pnt1 = new Point3D(firstPoint.X, firstPoint.Y, firstPoint.Z);
                pnt2 = new Point3D(secondPoint.X, firstPoint.Y, firstPoint.Z);
                pnt3 = new Point3D(secondPoint.X, secondPoint.Y, firstPoint.Z);
                pnt4 = new Point3D(firstPoint.X, secondPoint.Y, firstPoint.Z);
            }

            MakeRectangle(pnt1, pnt2, pnt3, pnt4);
        }

        if (!_shape.IsNull)
        {
            Done();
            return;
        }

        // box is a valid shape
        InternalSolid();
    }

    // Create a vertex if thin box in all directions is a point.
    private void MakeVertex(in Point3D point)
    {
        _shape = new MakeVertex(point).InternalVertex();
    }

    // Create an edge if thin box in two directions is a point.
    private void MakeEdge(in Point3D point1, in Point3D point2)
    {
        _shape = new MakeEdge(point1, point2).InternalEdge();
    }

    // Create a rectangular face if the box is thin in one direction only.
    private void MakeRectangle(in Point3D pnt1, in Point3D pnt2, in Point3D pnt3, in Point3D pnt4)
    {
        Edge edge1 = new MakeEdge(pnt1, pnt2).InternalEdge();
        Edge edge2 = new MakeEdge(pnt2, pnt3).InternalEdge();
        Edge edge3 = new MakeEdge(pnt3, pnt4).InternalEdge();
        Edge edge4 = new MakeEdge(pnt4, pnt1).InternalEdge();

        MakeWire wire = new(edge1, edge2, edge3, edge4);
        MakeFace face = new(wire.InternalWire());
        _shape = face.InternalShape();
    }
}
