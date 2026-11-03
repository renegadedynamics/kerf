// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_NurbsConvert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_NurbsConvert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Conversion of the whole geometry of a shape into NURBS geometry (planes excepted): the curves of its edges become
/// B-spline curves and the surfaces of its faces B-spline surfaces.
/// </summary>
/// <remarks>
/// After the modifier (<see cref="NurbsConvertModification"/>), the vertices of the edges whose tolerance
/// the conversion raised get that tolerance: new vertices of the result are raised in place, and the input's own
/// vertices are replaced by new, more tolerant ones (the input isn't changed). The result is read unfrozen until then.
/// </remarks>
public class BRepBuilderAPI_NurbsConvert : BRepBuilderAPI_ModifyShape
{
    private readonly NCollection_DataMap<Shape, Shape> myVtxToReplace =
        new(ShapeHasher.Instance);
    private readonly ReShape mySubs = new();

    /// <summary>A framework for converting a shape; give the shape with <see cref="Perform"/>.</summary>
    public BRepBuilderAPI_NurbsConvert()
    {
        myModification = new NurbsConvertModification();
    }

    /// <summary>Converts the geometry of <paramref name="S"/> into NURBS geometry.</summary>
    /// <param name="S">The shape.</param>
    /// <param name="Copy">Unused (as in OCCT).</param>
    public BRepBuilderAPI_NurbsConvert(in Shape S, bool Copy = false)
    {
        myModification = new NurbsConvertModification();
        Perform(S, Copy);
    }

    /// <summary>Converts the geometry of <paramref name="S"/> into NURBS geometry.</summary>
    /// <param name="S">The shape.</param>
    /// <param name="Copy">Unused (as in OCCT).</param>
    public void Perform(in Shape S, bool Copy = false)
    {
        DoModif(S, myModification!);
        CorrectVertexTol();
    }

    private void CorrectVertexTol()
    {
        var anInitVertices = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var anExp = new Explorer(myInitialShape, ShapeType.Vertex);
        for (; anExp.More(); anExp.Next())
        {
            anInitVertices.Add(anExp.Current());
        }

        var aModif = (NurbsConvertModification)myModification!;
        var aBB = new BrepBuilder();
        myVtxToReplace.Clear();
        var anEIter = new NCollection_List<Shape>.Iterator(aModif.GetUpdatedEdges());
        for (; anEIter.More(); anEIter.Next())
        {
            Shape anE = anEIter.Value();

            double anETol = Tool.Tolerance(ShapeCasts.Edge(anE));
            var anIter = new Iterator(anE);
            for (; anIter.More(); anIter.Next())
            {
                Vertex aVtx = ShapeCasts.Vertex(anIter.Value());
                if (anInitVertices.Contains(aVtx))
                {
                    if (myVtxToReplace.IsBound(aVtx))
                    {
                        aBB.UpdateVertex(ShapeCasts.Vertex(myVtxToReplace[aVtx]), anETol + Epsilon(anETol));
                    }
                    else
                    {
                        double aVTol = Tool.Tolerance(aVtx);
                        if (aVTol < anETol)
                        {
                            Vertex aNewVtx = new();
                            Point3D aVPnt = Tool.Pnt(aVtx);
                            aBB.MakeVertex(ref aNewVtx, aVPnt, anETol + Epsilon(anETol));
                            aNewVtx.Orientation(aVtx.Orientation());
                            myVtxToReplace.Bind(aVtx, aNewVtx);
                        }
                    }
                }
                else
                {
                    // a vertex the modifier made: unfrozen until Shape()
                    aBB.UpdateVertex(aVtx, anETol + Epsilon(anETol));
                }
            }
        }

        if (myVtxToReplace.IsEmpty())
        {
            return;
        }

        mySubs.Clear();
        foreach (KeyValuePair<Shape, Shape> anIter2 in myVtxToReplace)
        {
            mySubs.Replace(anIter2.Key, anIter2.Value);
        }
        mySubs.InternalApply(myShape);
        myShape = mySubs.InternalValue(myShape);
    }

    internal override Shape InternalModifiedShape(in Shape S)
    {
        if (S.ShapeType() == ShapeType.Vertex)
        {
            if (myVtxToReplace.IsBound(S))
            {
                return myVtxToReplace[S];
            }
        }
        if (myVtxToReplace.IsEmpty())
        {
            return myModifier.InternalModifiedShape(S);
        }
        else
        {
            Shape aNS = myModifier.InternalModifiedShape(S);
            return mySubs.InternalValue(aNS);
        }
    }

    internal override NCollection_List<Shape> InternalModified(in Shape F)
    {
        myGenerated.Clear();
        if (F.ShapeType() == ShapeType.Vertex)
        {
            if (myVtxToReplace.IsBound(F))
            {
                myGenerated.Append(myVtxToReplace[F]);
            }
            else
            {
                myGenerated.Append(myModifier.InternalModifiedShape(F));
            }
        }
        else
        {
            if (myVtxToReplace.IsEmpty())
            {
                myGenerated.Append(myModifier.InternalModifiedShape(F));
            }
            else
            {
                Shape aNS = myModifier.InternalModifiedShape(F);
                myGenerated.Append(mySubs.InternalValue(aNS));
            }
        }
        return myGenerated;
    }
}
