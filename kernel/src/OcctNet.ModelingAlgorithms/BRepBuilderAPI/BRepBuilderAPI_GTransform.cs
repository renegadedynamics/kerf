// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_GTransform.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_GTransform.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// General geometric transformation (a <see cref="gp_GTrsf"/>: an affinity, or any transformation defined by a 3 × 4
/// matrix) of a shape. The shape is first converted to NURBS geometry (<see cref="BRepBuilderAPI_NurbsConvert"/>),
/// then its curves and surfaces are transformed (<see cref="BRepTools_GTrsfModification"/>): a circle may become an
/// ellipse.
/// </summary>
public class BRepBuilderAPI_GTransform : BRepBuilderAPI_ModifyShape
{
    private readonly gp_GTrsf myGTrsf;
    private readonly BRepBuilderAPI_Collect myHist = new();

    /// <summary>A framework applying <paramref name="T"/>; give the shape with <see cref="Perform"/>.</summary>
    public BRepBuilderAPI_GTransform(in gp_GTrsf T)
    {
        myGTrsf = T;
        myModification = new BRepTools_GTrsfModification(T);
    }

    /// <summary>Applies <paramref name="T"/> to <paramref name="S"/>.</summary>
    /// <param name="S">The shape.</param>
    /// <param name="T">The transformation.</param>
    /// <param name="Copy">Passed to the NURBS conversion, which ignores it.</param>
    public BRepBuilderAPI_GTransform(in TopoDS_Shape S, in gp_GTrsf T, bool Copy = false)
    {
        myGTrsf = T;
        myModification = new BRepTools_GTrsfModification(T);
        Perform(S, Copy);
    }

    /// <summary>Applies the transformation to <paramref name="S"/>.</summary>
    public void Perform(in TopoDS_Shape S, bool Copy = false)
    {
        var nc = new BRepBuilderAPI_NurbsConvert();
        nc.Perform(S, Copy);
        myHist.Add(S, nc);
        // OCCT.NET: the converted shape is only an intermediate, read without freezing it
        TopoDS_Shape Slocal = nc.InternalShape();
        var theModif = (BRepTools_GTrsfModification)myModification!;
        theModif.GTrsf() = myGTrsf;
        DoModif(Slocal, myModification!);
        //  myHist.Filter (Shape());
    }

    internal override NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape F)
    {
        myGenerated.Clear();
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> M = myHist.Modification();
        if (M.IsBound(F))
        {
            // OCCT copies each image's list into a local that it never reads; the list returned is the one the last
            // base Modified call filled (the image of the last converted shape).
            var Li = new NCollection_List<TopoDS_Shape>();
            var itL = new NCollection_List<TopoDS_Shape>.Iterator(M[F]);
            for (; itL.More(); itL.Next())
            {
                Li.Assign(base.InternalModified(itL.Value()));
            }
        }
        return myGenerated;
    }

    internal override TopoDS_Shape InternalModifiedShape(in TopoDS_Shape S)
    {
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMapModif = myHist.Modification();
        TopoDS_Shape aShape = S;

        if (aMapModif.IsBound(S))
        {
            NCollection_List<TopoDS_Shape> aListModShape = aMapModif[S];
            int aNbShapes = aListModShape.Extent();

            if (aNbShapes > 0)
            {
                aShape = aListModShape.First();
            }
        }

        return base.InternalModifiedShape(aShape);
    }
}
