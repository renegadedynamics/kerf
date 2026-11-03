// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Geometry;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// GeometryTest_CurveCommands.cxx's <c>intersect</c>: surface/surface intersection with GeomInt_IntSS (approximated
/// lines; a start point, bounds, or both), and curve/surface intersection with GeomAPI_IntCS.
/// </summary>
internal static class GeometryTestIntersectCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "intersect", Intersection);
    }

    private static int Intersection(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        Curve3D? GC1 = null;
        Surface? GS1 = DrawTrSurf.GetSurface(theDI, a[2]);
        if (GS1 is null)
        {
            GC1 = DrawTrSurf.GetCurve(theDI, a[2]);
            if (GC1 is null)
            {
                return 1;
            }
        }

        Surface? GS2 = DrawTrSurf.GetSurface(theDI, a[3]);
        if (GS2 is null)
        {
            return 1;
        }

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (n == 5 || n == 9 || n == 13 || n == 17)
        {
            tol = theDI.Atof(a[n - 1]);
        }

        Curve3D Result;
        Point3D Point;

        if (GC1 is null)
        {
            var Inters = new IntSS();
            // Surface Surface
            if (n <= 5)
            {
                // General case
                Inters.Perform(GS1!, GS2, tol, true);
            }
            else if (n == 8 || n == 9 || n == 12 || n == 13 || n == 16 || n == 17)
            {
                bool useStart = true, useBnd = true;
                int ista1 = 0, ista2 = 0, ibnd1 = 0, ibnd2 = 0;
                double[] UVsta = new double[4];
                SurfaceAdaptor? AS1 = null, AS2 = null;

                if (n <= 9) // user starting point
                {
                    useBnd = false;
                    ista1 = 4;
                    ista2 = 7;
                }
                else if (n <= 13) // user bounding
                {
                    useStart = false;
                    ibnd1 = 4;
                    ibnd2 = 11;
                }
                else // both user starting point and bounding
                {
                    ista1 = 4;
                    ista2 = 7;
                    ibnd1 = 8;
                    ibnd2 = 15;
                }

                if (useStart)
                {
                    for (int i = ista1; i <= ista2; i++)
                    {
                        UVsta[i - ista1] = theDI.Atof(a[i]);
                    }
                }

                if (useBnd)
                {
                    double[] UVbnd = new double[8];
                    for (int i = ibnd1; i <= ibnd2; i++)
                    {
                        UVbnd[i - ibnd1] = theDI.Atof(a[i]);
                    }

                    AS1 = new SurfaceAdaptor(GS1!, UVbnd[0], UVbnd[1], UVbnd[2], UVbnd[3]);
                    AS2 = new SurfaceAdaptor(GS2, UVbnd[4], UVbnd[5], UVbnd[6], UVbnd[7]);
                }

                if (useStart && !useBnd)
                {
                    Inters.Perform(GS1!, GS2, tol, UVsta[0], UVsta[1], UVsta[2], UVsta[3]);
                }
                else if (!useStart && useBnd)
                {
                    Inters.Perform(AS1!, AS2!, tol);
                }
                else
                {
                    Inters.Perform(AS1!, AS2!, tol, UVsta[0], UVsta[1], UVsta[2], UVsta[3]);
                }
            }
            else
            {
                di.Append("incorrect number of arguments\n");
                return 1;
            }

            if (!Inters.IsDone)
            {
                di.Append("No intersections found!\n");
                return 1;
            }

            int aNbLines = Inters.LineCount;
            if (aNbLines >= 2)
            {
                for (int i = 1; i <= aNbLines; ++i)
                {
                    string buf = $"{a[1]}_{i}";
                    di.Append(buf).Append(" ");
                    Result = Inters.Line(i);
                    DrawTrSurf.Set(theDI, buf, Result);
                }
            }
            else if (aNbLines == 1)
            {
                Result = Inters.Line(1);
                di.Append(a[1]).Append(" ");
                DrawTrSurf.Set(theDI, a[1], Result);
            }

            int aNbPoints = Inters.PointCount;
            for (int i = 1; i <= aNbPoints; ++i)
            {
                Point = Inters.Point(i);
                string buf = $"{a[1]}_p_{i}";
                di.Append(buf).Append(" ");
                DrawTrSurf.Set(theDI, buf, Point);
            }
        }
        else
        {
            // Curve Surface
            var Inters = new CurveSurfaceIntersection(GC1, GS2);

            if (!Inters.IsDone)
            {
                di.Append("No intersections found!\n");
                return 1;
            }

            int nblines = Inters.SegmentCount;
            int nbpoints = Inters.PointCount;

            if ((nblines + nbpoints) >= 2)
            {
                int i;
                int Compt = 1;

                if (nblines >= 1)
                {
                    theDI.Output.Write("   Lines: \n");
                }

                for (i = 1; i <= nblines; i++, Compt++)
                {
                    string newname = $"{a[1]}_{Compt}";
                    di.Append(newname).Append(" ");
                    Result = Inters.Segment(i);
                    DrawTrSurf.Set(theDI, newname, Result);
                }

                if (nbpoints >= 1)
                {
                    theDI.Output.Write("   Points: \n");
                }

                int imax = nblines + nbpoints;

                for (/*i = 1*/; i <= imax; i++, Compt++)
                {
                    string newname = $"{a[1]}_{i}";
                    di.Append(newname).Append(" ");
                    // OCCT reads the point of index i, which runs from nblines + 1: past the points when there are
                    // segments too (Standard_OutOfRange, which Draw reports).
                    Point = Inters.Point(i);
                    DrawTrSurf.Set(theDI, newname, Point);
                }
            }
            else if (nblines == 1)
            {
                Result = Inters.Segment(1);
                di.Append(a[1]).Append(" ");
                DrawTrSurf.Set(theDI, a[1], Result);
            }
            else if (nbpoints == 1)
            {
                Point = Inters.Point(1);
                di.Append(a[1]).Append(" ");
                DrawTrSurf.Set(theDI, a[1], Point);
            }
        }

        return 0;
    }
}
