// Port of: src/ModelingAlgorithms/TKPrim/BRepPreviewAPI/BRepPreviewAPI_MakeBox.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPreviewAPI/BRepPreviewAPI_MakeBox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Builds a valid box if the points fulfil the conditions of a valid box, and a preview otherwise: a vertex if the box
/// is thin in all directions, an edge if it is thin in two, a rectangular face if it is thin in one.
/// </summary>
public class BRepPreviewAPI_MakeBox : BRepPrimAPI_MakeBox
{
    /// <summary>Constructor; set the box with <c>Init</c>.</summary>
    public BRepPreviewAPI_MakeBox()
    {
    }

    /// <summary>Creates a preview depending on point values.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        Point3D anLocation = myWedge.Axes().Location();

        Point3D aFirstPoint = new(anLocation.X(), anLocation.Y(), anLocation.Z());
        Point3D aSecondPoint = new(anLocation.X() + myWedge.GetXMax(),
                                  anLocation.Y() + myWedge.GetYMax(),
                                  anLocation.Z() + myWedge.GetZMax());

        bool aThinOnX = Math.Abs(aFirstPoint.X() - aSecondPoint.X()) < global::Kerf.Engine.Foundation.Precision.Confusion();
        bool aThinOnY = Math.Abs(aFirstPoint.Y() - aSecondPoint.Y()) < global::Kerf.Engine.Foundation.Precision.Confusion();
        bool aThinOnZ = Math.Abs(aFirstPoint.Z() - aSecondPoint.Z()) < global::Kerf.Engine.Foundation.Precision.Confusion();

        int aPreviewType = (aThinOnX ? 1 : 0) + (aThinOnY ? 1 : 0) + (aThinOnZ ? 1 : 0);

        if (aPreviewType == 3) // thin box in all directions is a point
        {
            makeVertex(aFirstPoint);
        }
        else if (aPreviewType == 2) // thin box in two directions is a point
        {
            makeEdge(aFirstPoint, aSecondPoint);
        }
        // thin box in only one direction is a rectangular face
        else if (aPreviewType == 1)
        {
            Point3D aPnt1 = new(), aPnt2 = new(), aPnt3 = new(), aPnt4 = new();
            if (aThinOnX)
            {
                aPnt1 = new Point3D(aFirstPoint.X(), aFirstPoint.Y(), aFirstPoint.Z());
                aPnt2 = new Point3D(aFirstPoint.X(), aSecondPoint.Y(), aFirstPoint.Z());
                aPnt3 = new Point3D(aFirstPoint.X(), aSecondPoint.Y(), aSecondPoint.Z());
                aPnt4 = new Point3D(aFirstPoint.X(), aFirstPoint.Y(), aSecondPoint.Z());
            }
            else if (aThinOnY)
            {
                aPnt1 = new Point3D(aFirstPoint.X(), aFirstPoint.Y(), aFirstPoint.Z());
                aPnt2 = new Point3D(aSecondPoint.X(), aFirstPoint.Y(), aFirstPoint.Z());
                aPnt3 = new Point3D(aSecondPoint.X(), aFirstPoint.Y(), aSecondPoint.Z());
                aPnt4 = new Point3D(aFirstPoint.X(), aFirstPoint.Y(), aSecondPoint.Z());
            }
            else if (aThinOnZ)
            {
                aPnt1 = new Point3D(aFirstPoint.X(), aFirstPoint.Y(), aFirstPoint.Z());
                aPnt2 = new Point3D(aSecondPoint.X(), aFirstPoint.Y(), aFirstPoint.Z());
                aPnt3 = new Point3D(aSecondPoint.X(), aSecondPoint.Y(), aFirstPoint.Z());
                aPnt4 = new Point3D(aFirstPoint.X(), aSecondPoint.Y(), aFirstPoint.Z());
            }

            makeRectangle(aPnt1, aPnt2, aPnt3, aPnt4);
        }

        if (!myShape.IsNull())
        {
            Done();
            return;
        }

        // box is a valid shape
        InternalSolid();
    }

    // Create a vertex if thin box in all directions is a point.
    private void makeVertex(in Point3D thePoint)
    {
        myShape = new BRepBuilderAPI_MakeVertex(thePoint).InternalVertex();
    }

    // Create an edge if thin box in two directions is a point.
    private void makeEdge(in Point3D thePoint1, in Point3D thePoint2)
    {
        myShape = new BRepBuilderAPI_MakeEdge(thePoint1, thePoint2).InternalEdge();
    }

    // Create a rectangular face if the box is thin in one direction only.
    private void makeRectangle(in Point3D thePnt1, in Point3D thePnt2, in Point3D thePnt3, in Point3D thePnt4)
    {
        TopoDS_Edge anEdge1 = new BRepBuilderAPI_MakeEdge(thePnt1, thePnt2).InternalEdge();
        TopoDS_Edge anEdge2 = new BRepBuilderAPI_MakeEdge(thePnt2, thePnt3).InternalEdge();
        TopoDS_Edge anEdge3 = new BRepBuilderAPI_MakeEdge(thePnt3, thePnt4).InternalEdge();
        TopoDS_Edge anEdge4 = new BRepBuilderAPI_MakeEdge(thePnt4, thePnt1).InternalEdge();

        BRepBuilderAPI_MakeWire aWire = new(anEdge1, anEdge2, anEdge3, anEdge4);
        BRepBuilderAPI_MakeFace aFace = new(aWire.InternalWire());
        myShape = aFace.InternalShape();
    }
}
