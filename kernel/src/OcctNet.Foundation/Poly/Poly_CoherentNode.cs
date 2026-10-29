// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentNode.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentNode.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A node of a <see cref="Poly_CoherentTriangulation"/>: a 3D point, UV coordinates, a normal, the list of triangles
/// using the node and an index (normally the node's index in the original triangulation).
/// </summary>
/// <remarks>
/// OCCT's node derives from gp_XYZ, a value type; here it is a class holding one (<see cref="XYZ"/>, with
/// <see cref="X"/>, <see cref="Y"/>, <see cref="Z"/> and <see cref="SetCoord"/> for convenience), since nodes are
/// elements of a linked structure edited in place. OCCT's allocator arguments are dropped.
/// </remarks>
public class Poly_CoherentNode
{
    private gp_XYZ myXYZ;
    private double myUV0;
    private double myUV1;
    private Poly_CoherentTriPtr? myTriangles;
    private int myIndex;
    private float myNormal0;
    private float myNormal1;
    private float myNormal2;

    /// <summary>A node at the origin, without UV coordinates, normal or triangles.</summary>
    /// <remarks>OCCT leaves the normal uninitialised here; the port's is zero (no normal).</remarks>
    public Poly_CoherentNode()
    {
        myXYZ = new gp_XYZ(0.0, 0.0, 0.0);
        myTriangles = null;
        myIndex = -1;
        myUV0 = Precision.Infinite();
        myUV1 = Precision.Infinite();
        // OCCT leaves myNormal uninitialised
    }

    /// <summary>A node at a point, without UV coordinates, normal or triangles.</summary>
    public Poly_CoherentNode(in gp_XYZ thePnt)
    {
        myXYZ = thePnt;
        myTriangles = null;
        myIndex = -1;
        myUV0 = Precision.Infinite();
        myUV1 = Precision.Infinite();
        myNormal0 = 0.0f;
        myNormal1 = 0.0f;
        myNormal2 = 0.0f;
    }

    /// <summary>The point.</summary>
    public gp_XYZ XYZ() => myXYZ;

    public double X() => myXYZ.X();

    public double Y() => myXYZ.Y();

    public double Z() => myXYZ.Z();

    /// <summary>Sets the point.</summary>
    public void SetCoord(double theX, double theY, double theZ) => myXYZ = new gp_XYZ(theX, theY, theZ);

    /// <summary>Sets the UV coordinates.</summary>
    public void SetUV(double theU, double theV)
    {
        myUV0 = theU;
        myUV1 = theV;
    }

    /// <summary>The U coordinate (infinite if not set).</summary>
    public double GetU() => myUV0;

    /// <summary>The V coordinate (infinite if not set).</summary>
    public double GetV() => myUV1;

    /// <summary>Sets the normal, rounded to single precision.</summary>
    public void SetNormal(in gp_XYZ theVector)
    {
        myNormal0 = (float)theVector.X();
        myNormal1 = (float)theVector.Y();
        myNormal2 = (float)theVector.Z();
    }

    /// <summary>True if the node has a normal (of square length above Precision.Confusion()).</summary>
    public bool HasNormal()
        => (myNormal0 * myNormal0 + myNormal1 * myNormal1 + myNormal2 * myNormal2) > Precision.Confusion();

    /// <summary>The normal.</summary>
    public gp_XYZ GetNormal() => new(myNormal0, myNormal1, myNormal2);

    /// <summary>Sets the index.</summary>
    public void SetIndex(int theIndex) => myIndex = theIndex;

    /// <summary>The index.</summary>
    public int GetIndex() => myIndex;

    /// <summary>True if no triangle uses this node.</summary>
    public bool IsFreeNode() => myTriangles is null;

    /// <summary>Resets the node to the origin, without UV coordinates or normal, and releases its list of triangles.</summary>
    /// <remarks>
    /// OCCT frees the list but keeps pointing to it, so the node is not free afterwards (and iterating its triangles
    /// reads freed memory). The port keeps the list.
    /// </remarks>
    public void Clear()
    {
        Poly_CoherentTriPtr.RemoveList(myTriangles); // OCCT: myTriangles is not reset
        myUV0 = Precision.Infinite();
        myUV1 = Precision.Infinite();
        myNormal0 = 0.0f;
        myNormal1 = 0.0f;
        myNormal2 = 0.0f;
        SetCoord(0.0, 0.0, 0.0);
    }

    /// <summary>Connects a triangle to this node.</summary>
    public void AddTriangle(Poly_CoherentTriangle theTri)
    {
        if (myTriangles is null)
        {
            myTriangles = new Poly_CoherentTriPtr(theTri);
        }
        else
        {
            myTriangles.Prepend(theTri);
        }
    }

    /// <summary>Disconnects a triangle from this node; false if it wasn't connected.</summary>
    public bool RemoveTriangle(Poly_CoherentTriangle theTri)
    {
        bool aResult = false;
        if (myTriangles!.GetTriangle() == theTri)
        {
            Poly_CoherentTriPtr aLostPtr = myTriangles;
            if (myTriangles == myTriangles.Next())
            {
                myTriangles = null;
            }
            else
            {
                myTriangles = myTriangles.Next();
            }

            Poly_CoherentTriPtr.Remove(aLostPtr);
            aResult = true;
        }
        else
        {
            var anIter = new Poly_CoherentTriPtr.Iterator(myTriangles);
            for (anIter.Next(); anIter.More(); anIter.Next())
            {
                if (anIter.Value() == theTri)
                {
                    Poly_CoherentTriPtr.Remove(anIter.PtrValue());
                    aResult = true;
                    break;
                }
            }
        }

        return aResult;
    }

    /// <summary>An iterator of the triangles using this node.</summary>
    public Poly_CoherentTriPtr.Iterator TriangleIterator() => new(myTriangles);

    /// <summary>Writes the point and the triangles of this node.</summary>
    public void Dump(TextWriter theStream)
    {
        theStream.Write("  X =" + F94(X()) + "; Y =" + F94(Y()) + "; Z =" + F94(Z()) + '\n');
        var anIter = new Poly_CoherentTriPtr.Iterator(myTriangles);
        for (; anIter.More(); anIter.Next())
        {
            Poly_CoherentTriangle aTri = anIter.Value();
            theStream.Write("      " + D5(aTri.Node(0)) + " " + D5(aTri.Node(1)) + " " + D5(aTri.Node(2)) + '\n');
        }
    }

    // printf's %9.4f and %5d
    private static string F94(double theValue) => theValue.ToString("F4", CultureInfo.InvariantCulture).PadLeft(9);

    private static string D5(int theValue) => theValue.ToString(CultureInfo.InvariantCulture).PadLeft(5);

    public override string ToString() => $"Poly_CoherentNode({myXYZ}, index {myIndex})";
}
