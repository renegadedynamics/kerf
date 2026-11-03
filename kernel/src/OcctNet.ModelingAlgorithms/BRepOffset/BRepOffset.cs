// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.BRepOffset_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Auxiliary tools for the offset algorithms.</summary>
public static class BRepOffset
{
    //=================================================================================================

    /// <summary>
    /// The surface offset from <paramref name="Surface"/> by <paramref name="Offset"/>: of its own type where possible
    /// (the offset of a plane is a plane), otherwise a Geom_OffsetSurface, made with <paramref name="allowC0"/>.
    /// </summary>
    public static Geom_Surface Surface(Geom_Surface Surface, double Offset, out BRepOffset_Status theStatus,
                                       bool allowC0 = false)
    {
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        theStatus = BRepOffset_Good;
        Geom_Surface? Result = null;

        Type TheType = Surface.GetType();

        if (TheType == typeof(Geom_Plane))
        {
            Geom_Plane P = (Geom_Plane)Surface;
            Vector3D T = new(P.Position().XDirection() ^ P.Position().YDirection());
            T *= Offset;
            Result = P.Translated(T) as Geom_Plane;
        }
        else if (TheType == typeof(Geom_CylindricalSurface))
        {
            Geom_CylindricalSurface C = (Geom_CylindricalSurface)Surface;
            double Radius = C.Radius();
            CoordinateSystem Axis = C.Position();
            if (Axis.Direct())
            {
                Radius += Offset;
            }
            else
            {
                Radius -= Offset;
            }
            if (Radius >= Tol)
            {
                Result = new Geom_CylindricalSurface(Axis, Radius);
            }
            else if (Radius <= -Tol)
            {
                Axis.Rotate(new Axis3D(Axis.Location(), Axis.Direction()), M_PI);
                Result = new Geom_CylindricalSurface(Axis, Math.Abs(Radius));
                theStatus = BRepOffset_Reversed;
            }
            else
            {
                theStatus = BRepOffset_Degenerated;
            }
        }
        else if (TheType == typeof(Geom_ConicalSurface))
        {
            Geom_ConicalSurface C = (Geom_ConicalSurface)Surface;
            double Alpha = C.SemiAngle();
            double Radius = C.RefRadius() + Offset * Math.Cos(Alpha);
            CoordinateSystem Axis = C.Position();
            if (Radius >= 0.0)
            {
                Vector3D Z = new(Axis.Direction());
                Z *= -Offset * Math.Sin(Alpha);
                Axis.Translate(Z);
            }
            else
            {
                Radius = -Radius;
                Vector3D Z = new(Axis.Direction());
                Z *= -Offset * Math.Sin(Alpha);
                Axis.Translate(Z);
                Axis.Rotate(new Axis3D(Axis.Location(), Axis.Direction()), M_PI);
                Alpha = -Alpha;
            }
            Result = new Geom_ConicalSurface(Axis, Alpha, Radius);
        }
        else if (TheType == typeof(Geom_SphericalSurface))
        {
            Geom_SphericalSurface S = (Geom_SphericalSurface)Surface;
            double Radius = S.Radius();
            CoordinateSystem Axis = S.Position();
            if (Axis.Direct())
            {
                Radius += Offset;
            }
            else
            {
                Radius -= Offset;
            }
            if (Radius >= Tol)
            {
                Result = new Geom_SphericalSurface(Axis, Radius);
            }
            else if (Radius <= -Tol)
            {
                Axis.Rotate(new Axis3D(Axis.Location(), Axis.Direction()), M_PI);
                Axis.ZReverse();
                Result = new Geom_SphericalSurface(Axis, -Radius);
                theStatus = BRepOffset_Reversed;
            }
            else
            {
                theStatus = BRepOffset_Degenerated;
            }
        }
        else if (TheType == typeof(Geom_ToroidalSurface))
        {
            Geom_ToroidalSurface S = (Geom_ToroidalSurface)Surface;
            double MajorRadius = S.MajorRadius();
            double MinorRadius = S.MinorRadius();
            CoordinateSystem Axis = S.Position();
            if (MinorRadius < MajorRadius)
            { // A FINIR
                if (Axis.Direct())
                {
                    MinorRadius += Offset;
                }
                else
                {
                    MinorRadius -= Offset;
                }
                if (MinorRadius >= Tol)
                {
                    Result = new Geom_ToroidalSurface(Axis, MajorRadius, MinorRadius);
                }
                else if (MinorRadius <= -Tol)
                {
                    theStatus = BRepOffset_Reversed;
                }
                else
                {
                    theStatus = BRepOffset_Degenerated;
                }
            }
        }
        else if (TheType == typeof(Geom_SurfaceOfRevolution))
        {
        }
        else if (TheType == typeof(Geom_SurfaceOfLinearExtrusion))
        {
        }
        else if (TheType == typeof(Geom_BSplineSurface))
        {
        }
        else if (TheType == typeof(Geom_RectangularTrimmedSurface))
        {
            Geom_RectangularTrimmedSurface S = (Geom_RectangularTrimmedSurface)Surface;
            S.Bounds(out double U1, out double U2, out double V1, out double V2);
            Geom_Surface Off = BRepOffset.Surface(S.BasisSurface(), Offset, out theStatus, allowC0);
            Result = new Geom_RectangularTrimmedSurface(Off, U1, U2, V1, V2);
        }
        else if (TheType == typeof(Geom_OffsetSurface))
        {
        }

        if (Result is null)
        {
            Result = new Geom_OffsetSurface(Surface, Offset, allowC0);
        }

        return Result;
    }

    //=================================================================================================

    /// <summary>
    /// Preprocesses a surface to be offset (a B-spline; OCCT plans Béziers and revolutions of them) by collapsing each
    /// singular side to a single point, so that the normal doesn't flip at the singularity. A side is singular if the
    /// face has a degenerated edge whose vertex holds (by its tolerance) every pole of that side, or if all its poles
    /// fit in a sphere of radius <paramref name="thePrecision"/>. Returns the surface itself or a modified copy.
    /// </summary>
    public static Geom_Surface CollapseSingularities(Geom_Surface theSurface, in TopoDS_Face theFace,
                                                     double thePrecision)
    {
        // check surface type to see if it can be processed
        Type aType = theSurface.GetType();
        if (aType != typeof(Geom_BSplineSurface))
        {
            // for the moment, only bspline surfaces are treated;
            // in the future, bezier surfaces and surfaces of revolution can be also handled
            return theSurface;
        }

        // find singularities (vertices of degenerated edges)
        NCollection_List<Point3D> aDegenPnt = new();
        NCollection_List<double> aDegenTol = new();
        for (TopExp_Explorer anExp = new(theFace, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
            if (!BRep_Tool.Degenerated(anEdge))
            {
                continue;
            }
            TopExp.Vertices(anEdge, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            if (!aV1.IsSame(aV2))
            {
                continue;
            }

            aDegenPnt.Append(BRep_Tool.Pnt(aV1));
            aDegenTol.Append(BRep_Tool.Tolerance(aV1));
        }

        // iterate by sides of the surface
        if (aType == typeof(Geom_BSplineSurface))
        {
            Geom_BSplineSurface aBSpline = (Geom_BSplineSurface)theSurface;
            NCollection_Array2<Point3D> aPoles = aBSpline.InternalPoles();

            Geom_BSplineSurface? aCopy = null;

            // iterate by sides: {U=0; V=0; U=1; V=1}
            int[] RowStart = [aPoles.LowerRow(), aPoles.LowerRow(), aPoles.UpperRow(), aPoles.LowerRow()];
            int[] ColStart = [aPoles.LowerCol(), aPoles.LowerCol(), aPoles.LowerCol(), aPoles.UpperCol()];
            int[] RowStep = [0, 1, 0, 1];
            int[] ColStep = [1, 0, 1, 0];
            int[] NbSteps = [aPoles.RowLength(), aPoles.ColLength(), aPoles.RowLength(), aPoles.ColLength()];
            for (int iSide = 0; iSide < 4; iSide++)
            {
                // compute center of gravity of side poles
                Xyz aSum = new();
                for (int iPole = 0; iPole < NbSteps[iSide]; iPole++)
                {
                    aSum += aPoles[RowStart[iSide] + iPole * RowStep[iSide], ColStart[iSide] + iPole * ColStep[iSide]]
                        .XYZ();
                }
                Point3D aCenter = new(aSum / NbSteps[iSide]);

                // determine if all poles of the side fit into:
                bool isCollapsed = true; // aCenter precisely (with gp::Resolution())
                bool isSingular = true;  // aCenter with thePrecision
                bool[] isDegenerated = new bool[aDegenPnt.Extent()]; // degenerated vertex
                for (int iDegen = 0; iDegen < isDegenerated.Length; ++iDegen)
                {
                    isDegenerated[iDegen] = true;
                }
                for (int iPole = 0; iPole < NbSteps[iSide]; iPole++)
                {
                    Point3D aPole = aPoles[RowStart[iSide] + iPole * RowStep[iSide],
                                          ColStart[iSide] + iPole * ColStep[iSide]];

                    // distance from CG
                    double aDistCG = aCenter.Distance(aPole);
                    if (aDistCG > Constants.Resolution())
                    {
                        isCollapsed = false;
                    }
                    if (aDistCG > thePrecision)
                    {
                        isSingular = false;
                    }

                    // distances from degenerated points
                    NCollection_List<Point3D>.Iterator aDegPntIt = new(aDegenPnt);
                    NCollection_List<double>.Iterator aDegTolIt = new(aDegenTol);
                    for (int iDegen = 0; iDegen < isDegenerated.Length;
                         aDegPntIt.Next(), aDegTolIt.Next(), ++iDegen)
                    {
                        if (isDegenerated[iDegen] && aDegPntIt.Value().Distance(aPole) >= aDegTolIt.Value())
                        {
                            isDegenerated[iDegen] = false;
                        }
                    }
                }
                if (isCollapsed)
                {
                    continue; // already Ok, nothing to be done
                }

                // decide to collapse the side: either if it is singular with thePrecision,
                // or if it fits into one (and only one) degenerated point
                if (!isSingular)
                {
                    int aNbFit = 0;
                    NCollection_List<Point3D>.Iterator aDegPntIt = new(aDegenPnt);
                    NCollection_List<double>.Iterator aDegTolIt = new(aDegenTol);
                    for (int iDegen = 0; iDegen < isDegenerated.Length; ++iDegen)
                    {
                        if (isDegenerated[iDegen])
                        {
                            // remove degenerated point as soon as it fits at least one side, to prevent total
                            // collapse
                            aDegenPnt.Remove(aDegPntIt);
                            aDegenTol.Remove(aDegTolIt);
                            aNbFit++;
                        }
                        else
                        {
                            aDegPntIt.Next();
                            aDegTolIt.Next();
                        }
                    }

                    // if side fits more than one degenerated vertex, do not collapse it
                    // to be on the safe side
                    isSingular = (aNbFit == 1);
                }

                // do collapse
                if (isSingular)
                {
                    if (aCopy is null)
                    {
                        aCopy = (Geom_BSplineSurface)theSurface.Copy();
                    }
                    for (int iPole = 0; iPole < NbSteps[iSide]; iPole++)
                    {
                        aCopy.SetPole(RowStart[iSide] + iPole * RowStep[iSide],
                                      ColStart[iSide] + iPole * ColStep[iSide],
                                      aCenter);
                    }
                }
            }

            if (aCopy is not null)
            {
                return aCopy;
            }
        }

        return theSurface;
    }
}
