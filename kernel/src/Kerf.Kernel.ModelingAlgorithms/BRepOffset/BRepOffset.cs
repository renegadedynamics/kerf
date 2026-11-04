// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Offsets.OffsetStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Offsets;

/// <summary>Auxiliary tools for the offset algorithms.</summary>
internal static class BRepOffset
{
    //=================================================================================================

    /// <summary>
    /// The surface offset from <paramref name="surface"/> by <paramref name="offset"/>: of its own type where possible
    /// (the offset of a plane is a plane), otherwise a Geom_OffsetSurface, made with <paramref name="allowC0"/>.
    /// </summary>
    public static Surface Surface(Surface surface, double offset, out OffsetStatus status,
                                       bool allowC0 = false)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        status = OffsetStatus.Good;
        Surface? result = null;

        Type theType = surface.GetType();

        if (theType == typeof(PlaneSurface))
        {
            PlaneSurface p = (PlaneSurface)surface;
            Vector3D t = new(p.Position.XDirection ^ p.Position.YDirection);
            t *= offset;
            result = p.Translated(t) as PlaneSurface;
        }
        else if (theType == typeof(CylindricalSurface))
        {
            CylindricalSurface c = (CylindricalSurface)surface;
            double radius = c.Radius;
            CoordinateSystem axis = c.Position;
            if (axis.IsDirect)
            {
                radius += offset;
            }
            else
            {
                radius -= offset;
            }
            if (radius >= tol)
            {
                result = new CylindricalSurface(axis, radius);
            }
            else if (radius <= -tol)
            {
                axis = axis.Rotated(new Axis3D(axis.Location, axis.Direction), M_PI);
                result = new CylindricalSurface(axis, Math.Abs(radius));
                status = OffsetStatus.Reversed;
            }
            else
            {
                status = OffsetStatus.Degenerated;
            }
        }
        else if (theType == typeof(ConicalSurface))
        {
            ConicalSurface c = (ConicalSurface)surface;
            double alpha = c.SemiAngle;
            double radius = c.RefRadius + offset * Math.Cos(alpha);
            CoordinateSystem axis = c.Position;
            if (radius >= 0.0)
            {
                Vector3D z = new(axis.Direction);
                z *= -offset * Math.Sin(alpha);
                axis = axis.Translated(z);
            }
            else
            {
                radius = -radius;
                Vector3D z = new(axis.Direction);
                z *= -offset * Math.Sin(alpha);
                axis = axis.Translated(z);
                axis = axis.Rotated(new Axis3D(axis.Location, axis.Direction), M_PI);
                alpha = -alpha;
            }
            result = new ConicalSurface(axis, alpha, radius);
        }
        else if (theType == typeof(SphericalSurface))
        {
            SphericalSurface s = (SphericalSurface)surface;
            double radius = s.Radius;
            CoordinateSystem axis = s.Position;
            if (axis.IsDirect)
            {
                radius += offset;
            }
            else
            {
                radius -= offset;
            }
            if (radius >= tol)
            {
                result = new SphericalSurface(axis, radius);
            }
            else if (radius <= -tol)
            {
                axis = axis.Rotated(new Axis3D(axis.Location, axis.Direction), M_PI);
                axis = axis.ZReversed();
                result = new SphericalSurface(axis, -radius);
                status = OffsetStatus.Reversed;
            }
            else
            {
                status = OffsetStatus.Degenerated;
            }
        }
        else if (theType == typeof(ToroidalSurface))
        {
            ToroidalSurface s = (ToroidalSurface)surface;
            double majorRadius = s.MajorRadius;
            double minorRadius = s.MinorRadius;
            CoordinateSystem axis = s.Position;
            if (minorRadius < majorRadius)
            { // A FINIR
                if (axis.IsDirect)
                {
                    minorRadius += offset;
                }
                else
                {
                    minorRadius -= offset;
                }
                if (minorRadius >= tol)
                {
                    result = new ToroidalSurface(axis, majorRadius, minorRadius);
                }
                else if (minorRadius <= -tol)
                {
                    status = OffsetStatus.Reversed;
                }
                else
                {
                    status = OffsetStatus.Degenerated;
                }
            }
        }
        else if (theType == typeof(RevolutionSurface))
        {
        }
        else if (theType == typeof(ExtrusionSurface))
        {
        }
        else if (theType == typeof(BSplineSurface))
        {
        }
        else if (theType == typeof(TrimmedSurface))
        {
            TrimmedSurface s = (TrimmedSurface)surface;
            s.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
            Surface off = global::Kerf.Engine.Offsets.BRepOffset.Surface(s.BasisSurface, offset, out status, allowC0);
            result = new TrimmedSurface(off, u1, u2, v1, v2);
        }
        else if (theType == typeof(OffsetSurface))
        {
        }

        if (result is null)
        {
            result = new OffsetSurface(surface, offset, allowC0);
        }

        return result;
    }

    //=================================================================================================

    /// <summary>
    /// Preprocesses a surface to be offset (a B-spline; OCCT plans Béziers and revolutions of them) by collapsing each
    /// singular side to a single point, so that the normal doesn't flip at the singularity. A side is singular if the
    /// face has a degenerated edge whose vertex holds (by its tolerance) every pole of that side, or if all its poles
    /// fit in a sphere of radius <paramref name="precision"/>. Returns the surface itself or a modified copy.
    /// </summary>
    public static Surface CollapseSingularities(Surface surface, in Face face,
                                                     double precision)
    {
        // check surface type to see if it can be processed
        Type type = surface.GetType();
        if (type != typeof(BSplineSurface))
        {
            // for the moment, only bspline surfaces are treated;
            // in the future, bezier surfaces and surfaces of revolution can be also handled
            return surface;
        }

        // find singularities (vertices of degenerated edges)
        NCollection_List<Point3D> degenPnt = new();
        NCollection_List<double> degenTol = new();
        for (Explorer exp = new(face, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());
            if (!Tool.Degenerated(edge))
            {
                continue;
            }
            Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
            if (!v1.IsSame(v2))
            {
                continue;
            }

            degenPnt.Append(Tool.Pnt(v1));
            degenTol.Append(Tool.Tolerance(v1));
        }

        // iterate by sides of the surface
        if (type == typeof(BSplineSurface))
        {
            BSplineSurface bSpline = (BSplineSurface)surface;
            NCollection_Array2<Point3D> poles = bSpline.InternalPoles();

            BSplineSurface? copy = null;

            // iterate by sides: {U=0; V=0; U=1; V=1}
            int[] rowStart = [poles.LowerRow(), poles.LowerRow(), poles.UpperRow(), poles.LowerRow()];
            int[] colStart = [poles.LowerCol(), poles.LowerCol(), poles.LowerCol(), poles.UpperCol()];
            int[] rowStep = [0, 1, 0, 1];
            int[] colStep = [1, 0, 1, 0];
            int[] nbSteps = [poles.RowLength(), poles.ColLength(), poles.RowLength(), poles.ColLength()];
            for (int iSide = 0; iSide < 4; iSide++)
            {
                // compute center of gravity of side poles
                Xyz sum = new();
                for (int iPole = 0; iPole < nbSteps[iSide]; iPole++)
                {
                    sum += poles[rowStart[iSide] + iPole * rowStep[iSide], colStart[iSide] + iPole * colStep[iSide]]
                        .XYZ();
                }
                Point3D center = new(sum / nbSteps[iSide]);

                // determine if all poles of the side fit into:
                bool isCollapsed = true; // aCenter precisely (with gp::Resolution())
                bool isSingular = true;  // aCenter with thePrecision
                bool[] isDegenerated = new bool[degenPnt.Extent()]; // degenerated vertex
                for (int iDegen = 0; iDegen < isDegenerated.Length; ++iDegen)
                {
                    isDegenerated[iDegen] = true;
                }
                for (int iPole = 0; iPole < nbSteps[iSide]; iPole++)
                {
                    Point3D pole = poles[rowStart[iSide] + iPole * rowStep[iSide],
                                          colStart[iSide] + iPole * colStep[iSide]];

                    // distance from CG
                    double distCG = center.DistanceTo(pole);
                    if (distCG > Constants.Resolution())
                    {
                        isCollapsed = false;
                    }
                    if (distCG > precision)
                    {
                        isSingular = false;
                    }

                    // distances from degenerated points
                    NCollection_List<Point3D>.Iterator degPntIt = new(degenPnt);
                    NCollection_List<double>.Iterator degTolIt = new(degenTol);
                    for (int iDegen = 0; iDegen < isDegenerated.Length;
                         degPntIt.Next(), degTolIt.Next(), ++iDegen)
                    {
                        if (isDegenerated[iDegen] && degPntIt.Value().DistanceTo(pole) >= degTolIt.Value())
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
                    int nbFit = 0;
                    NCollection_List<Point3D>.Iterator degPntIt = new(degenPnt);
                    NCollection_List<double>.Iterator degTolIt = new(degenTol);
                    for (int iDegen = 0; iDegen < isDegenerated.Length; ++iDegen)
                    {
                        if (isDegenerated[iDegen])
                        {
                            // remove degenerated point as soon as it fits at least one side, to prevent total
                            // collapse
                            degenPnt.Remove(degPntIt);
                            degenTol.Remove(degTolIt);
                            nbFit++;
                        }
                        else
                        {
                            degPntIt.Next();
                            degTolIt.Next();
                        }
                    }

                    // if side fits more than one degenerated vertex, do not collapse it
                    // to be on the safe side
                    isSingular = (nbFit == 1);
                }

                // do collapse
                if (isSingular)
                {
                    if (copy is null)
                    {
                        copy = (BSplineSurface)surface.Copy();
                    }
                    for (int iPole = 0; iPole < nbSteps[iSide]; iPole++)
                    {
                        copy.SetPole(rowStart[iSide] + iPole * rowStep[iSide],
                                      colStart[iSide] + iPole * colStep[iSide],
                                      center);
                    }
                }
            }

            if (copy is not null)
            {
                return copy;
            }
        }

        return surface;
    }
}
