// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_GTransform.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_GTransform.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// General geometric transformation (a <see cref="GeneralTransform3D"/>: an affinity, or any transformation defined by a 3 × 4
/// matrix) of a shape. The shape is first converted to NURBS geometry (<see cref="BRepBuilderAPI_NurbsConvert"/>),
/// then its curves and surfaces are transformed (<see cref="GTrsfModification"/>): a circle may become an
/// ellipse.
/// </summary>
public class BRepBuilderAPI_GTransform : BRepBuilderAPI_ModifyShape
{
    private readonly GeneralTransform3D myGTrsf;
    private readonly BRepBuilderAPI_Collect myHist = new();

    /// <summary>A framework applying <paramref name="T"/>; give the shape with <see cref="Perform"/>.</summary>
    public BRepBuilderAPI_GTransform(in GeneralTransform3D T)
    {
        myGTrsf = T;
        myModification = new GTrsfModification(T);
    }

    /// <summary>Applies <paramref name="T"/> to <paramref name="S"/>.</summary>
    /// <param name="S">The shape.</param>
    /// <param name="T">The transformation.</param>
    /// <param name="Copy">Passed to the NURBS conversion, which ignores it.</param>
    public BRepBuilderAPI_GTransform(in Shape S, in GeneralTransform3D T, bool Copy = false)
    {
        myGTrsf = T;
        myModification = new GTrsfModification(T);
        Perform(S, Copy);
    }

    /// <summary>Applies the transformation to <paramref name="S"/>.</summary>
    public void Perform(in Shape S, bool Copy = false)
    {
        var nc = new BRepBuilderAPI_NurbsConvert();
        nc.Perform(S, Copy);
        myHist.Add(S, nc);
        // OCCT.NET: the converted shape is only an intermediate, read without freezing it
        Shape Slocal = nc.InternalShape();
        var theModif = (GTrsfModification)myModification!;
        theModif.GTrsf() = myGTrsf;
        DoModif(Slocal, myModification!);
        //  myHist.Filter (Shape());
    }

    internal override NCollection_List<Shape> InternalModified(in Shape F)
    {
        myGenerated.Clear();
        NCollection_DataMap<Shape, NCollection_List<Shape>> M = myHist.Modification();
        if (M.IsBound(F))
        {
            // OCCT copies each image's list into a local that it never reads; the list returned is the one the last
            // base Modified call filled (the image of the last converted shape).
            var Li = new NCollection_List<Shape>();
            var itL = new NCollection_List<Shape>.Iterator(M[F]);
            for (; itL.More(); itL.Next())
            {
                Li.Assign(base.InternalModified(itL.Value()));
            }
        }
        return myGenerated;
    }

    internal override Shape InternalModifiedShape(in Shape S)
    {
        NCollection_DataMap<Shape, NCollection_List<Shape>> aMapModif = myHist.Modification();
        Shape aShape = S;

        if (aMapModif.IsBound(S))
        {
            NCollection_List<Shape> aListModShape = aMapModif[S];
            int aNbShapes = aListModShape.Extent();

            if (aNbShapes > 0)
            {
                aShape = aListModShape.First();
            }
        }

        return base.InternalModifiedShape(aShape);
    }
}
