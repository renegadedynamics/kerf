// Port of: src/ModelingAlgorithms/TKPrim/BRepPreviewAPI/BRepPreviewAPI_MakeBox.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPreviewAPI/BRepPreviewAPI_MakeBox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    public override void Build(Message_ProgressRange theRange = default)
    {
        gp_Pnt anLocation = myWedge.Axes().Location();

        gp_Pnt aFirstPoint = new(anLocation.X(), anLocation.Y(), anLocation.Z());
        gp_Pnt aSecondPoint = new(anLocation.X() + myWedge.GetXMax(),
                                  anLocation.Y() + myWedge.GetYMax(),
                                  anLocation.Z() + myWedge.GetZMax());

        bool aThinOnX = Math.Abs(aFirstPoint.X() - aSecondPoint.X()) < Precision.Confusion();
        bool aThinOnY = Math.Abs(aFirstPoint.Y() - aSecondPoint.Y()) < Precision.Confusion();
        bool aThinOnZ = Math.Abs(aFirstPoint.Z() - aSecondPoint.Z()) < Precision.Confusion();

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
            gp_Pnt aPnt1 = new(), aPnt2 = new(), aPnt3 = new(), aPnt4 = new();
            if (aThinOnX)
            {
                aPnt1 = new gp_Pnt(aFirstPoint.X(), aFirstPoint.Y(), aFirstPoint.Z());
                aPnt2 = new gp_Pnt(aFirstPoint.X(), aSecondPoint.Y(), aFirstPoint.Z());
                aPnt3 = new gp_Pnt(aFirstPoint.X(), aSecondPoint.Y(), aSecondPoint.Z());
                aPnt4 = new gp_Pnt(aFirstPoint.X(), aFirstPoint.Y(), aSecondPoint.Z());
            }
            else if (aThinOnY)
            {
                aPnt1 = new gp_Pnt(aFirstPoint.X(), aFirstPoint.Y(), aFirstPoint.Z());
                aPnt2 = new gp_Pnt(aSecondPoint.X(), aFirstPoint.Y(), aFirstPoint.Z());
                aPnt3 = new gp_Pnt(aSecondPoint.X(), aFirstPoint.Y(), aSecondPoint.Z());
                aPnt4 = new gp_Pnt(aFirstPoint.X(), aFirstPoint.Y(), aSecondPoint.Z());
            }
            else if (aThinOnZ)
            {
                aPnt1 = new gp_Pnt(aFirstPoint.X(), aFirstPoint.Y(), aFirstPoint.Z());
                aPnt2 = new gp_Pnt(aSecondPoint.X(), aFirstPoint.Y(), aFirstPoint.Z());
                aPnt3 = new gp_Pnt(aSecondPoint.X(), aSecondPoint.Y(), aFirstPoint.Z());
                aPnt4 = new gp_Pnt(aFirstPoint.X(), aSecondPoint.Y(), aFirstPoint.Z());
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
    private void makeVertex(in gp_Pnt thePoint)
    {
        myShape = new BRepBuilderAPI_MakeVertex(thePoint).InternalVertex();
    }

    // Create an edge if thin box in two directions is a point.
    private void makeEdge(in gp_Pnt thePoint1, in gp_Pnt thePoint2)
    {
        myShape = new BRepBuilderAPI_MakeEdge(thePoint1, thePoint2).InternalEdge();
    }

    // Create a rectangular face if the box is thin in one direction only.
    private void makeRectangle(in gp_Pnt thePnt1, in gp_Pnt thePnt2, in gp_Pnt thePnt3, in gp_Pnt thePnt4)
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
