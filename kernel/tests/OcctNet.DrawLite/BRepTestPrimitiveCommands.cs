// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Preview;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace OcctNet.DrawLite;

/// <summary>
/// BRepTest's primitive commands (BRepTest_PrimitiveCommands.cxx): box (with -min, -max, -size, -dir, -xdir and
/// -preview), wedge, pcylinder, pcone, psphere and ptorus; and BRepTest_TopologyCommands' halfspace.
/// </summary>
/// <remarks>
/// Each makes its shape with the BRepPrimAPI maker Draw uses, through the maker's freezing accessor (the conversion
/// operator OCCT calls), and binds it. A command given the wrong number of words returns 1 with no message, as Draw.
/// <c>[plane(ax2)]</c> is a DrawTrSurf plane (DrawTrSurf::Get down-cast to Geom_Plane); a word that isn't one reads
/// as a number.
/// </remarks>
internal static class BRepTestPrimitiveCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "box", Box);
        DrawCxx.Add(theInterpreter, "wedge", Wedge);
        DrawCxx.Add(theInterpreter, "pcylinder", Cylinder);
        DrawCxx.Add(theInterpreter, "pcone", Cone);
        DrawCxx.Add(theInterpreter, "psphere", Sphere);
        DrawCxx.Add(theInterpreter, "ptorus", Torus);
        DrawCxx.Add(theInterpreter, "halfspace", HalfSpace);
    }

    private const double DegToRad = Math.PI / 180.0;

    /// <summary>DrawTrSurf::Get(name) down-cast to Geom_Plane: the plane bound to the name, or null.</summary>
    internal static PlaneSurface? GetPlane(DrawInterpreter theDI, string theName)
        => theDI.GetDrawable(theName) is DrawGeometry { Geometry: PlaneSurface aPlane } ? aPlane : null;

    // OCCT dereferences a null plane in these forms (pcone, psphere, ptorus with the most words); a .NET null
    // dereference stands for the access violation
    private static PlaneSurface NonNull(PlaneSurface? P) => P ?? throw new NullReferenceException("DrawTrSurf::Get: not a plane");

    private static int Box(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        Point3D anOrigin = new();
        Xyz aParams = new();
        Direction3D aDir = new();
        Direction3D aXDir = new();
        bool isMinMax = false;
        bool isPreview = false;
        bool isAxis = false;

        for (int anArgIter = 2; anArgIter < n; ++anArgIter)
        {
            string anArgCase = a[anArgIter].ToLowerInvariant();
            if (anArgCase == "-min" && anArgIter + 3 <= n)
            {
                anOrigin.SetX(theDI.Atof(a[anArgIter + 1]));
                anOrigin.SetY(theDI.Atof(a[anArgIter + 2]));
                anOrigin.SetZ(theDI.Atof(a[anArgIter + 3]));
                anArgIter += 3;
            }
            else if (anArgCase == "-max" && anArgIter + 3 <= n)
            {
                aParams.SetX(theDI.Atof(a[anArgIter + 1]));
                aParams.SetY(theDI.Atof(a[anArgIter + 2]));
                aParams.SetZ(theDI.Atof(a[anArgIter + 3]));
                isMinMax = true;
                anArgIter += 3;
            }
            else if (anArgCase == "-size" && anArgIter + 3 <= n)
            {
                aParams.SetX(theDI.Atof(a[anArgIter + 1]));
                aParams.SetY(theDI.Atof(a[anArgIter + 2]));
                aParams.SetZ(theDI.Atof(a[anArgIter + 3]));
                isMinMax = false;
                anArgIter += 3;
            }
            else if (anArgCase == "-dir" && anArgIter + 3 <= n)
            {
                double aX = theDI.Atof(a[anArgIter + 1]);
                double anY = theDI.Atof(a[anArgIter + 2]);
                double aZ = theDI.Atof(a[anArgIter + 3]);
                aDir.SetCoord(aX, anY, aZ);
                isAxis = true;
                anArgIter += 3;
            }
            else if (anArgCase == "-xdir" && anArgIter + 3 <= n)
            {
                double aX = theDI.Atof(a[anArgIter + 1]);
                double anY = theDI.Atof(a[anArgIter + 2]);
                double aZ = theDI.Atof(a[anArgIter + 3]);
                aXDir.SetCoord(aX, anY, aZ);
                isAxis = true;
                anArgIter += 3;
            }
            else if (anArgCase == "-preview")
            {
                isPreview = true;
            }
            else if (anArgIter + 5 < n || anArgIter + 2 < n)
            {
                int aCountReal = 0;
                int anIter = anArgIter;
                while (anIter < n && theDI.ParseReal(a[anIter], out _))
                {
                    anIter++;
                    aCountReal++;
                }

                if (aCountReal == 6)
                {
                    anOrigin.SetX(theDI.Atof(a[anArgIter]));
                    anOrigin.SetY(theDI.Atof(a[anArgIter + 1]));
                    anOrigin.SetZ(theDI.Atof(a[anArgIter + 2]));
                    aParams.SetX(theDI.Atof(a[anArgIter + 3]));
                    aParams.SetY(theDI.Atof(a[anArgIter + 4]));
                    aParams.SetZ(theDI.Atof(a[anArgIter + 5]));
                    anArgIter += 5;
                }
                else if (aCountReal == 3)
                {
                    aParams.SetX(theDI.Atof(a[anArgIter]));
                    aParams.SetY(theDI.Atof(a[anArgIter + 1]));
                    aParams.SetZ(theDI.Atof(a[anArgIter + 2]));
                    anArgIter += 2;
                }
                else
                {
                    theDI.Output.WriteLine("Syntax error");
                    return 1;
                }
            }
        }

        if (isPreview)
        {
            Shape S;
            var aPreview = new MakePreviewBox();
            if (isMinMax)
            {
                aPreview.Init(anOrigin, new Point3D(aParams));
            }
            else if (!isMinMax && !isAxis)
            {
                aPreview.Init(anOrigin, aParams.X(), aParams.Y(), aParams.Z());
            }
            else if (isAxis)
            {
                var anAxis = new Frame3D(anOrigin, aDir, aXDir);
                aPreview.Init(anAxis, aParams.X(), aParams.Y(), aParams.Z());
            }
            else
            {
                aPreview.Init(aParams.X(), aParams.Y(), aParams.Z());
            }

            S = aPreview.Shape();
            theDI.SetShape(a[1], S);
        }
        else
        {
            Solid S;
            if (isMinMax)
            {
                S = new MakeBox(anOrigin, new Point3D(aParams)).Solid();
            }
            else if (!isMinMax && !isAxis)
            {
                S = new MakeBox(anOrigin, aParams.X(), aParams.Y(), aParams.Z()).Solid();
            }
            else if (isAxis)
            {
                var anAxis = new Frame3D(anOrigin, aDir, aXDir);
                S = new MakeBox(anAxis, aParams.X(), aParams.Y(), aParams.Z()).Solid();
            }
            else
            {
                S = new MakeBox(aParams.X(), aParams.Y(), aParams.Z()).Solid();
            }

            theDI.SetShape(a[1], S);
        }

        return 0;
    }

    private static int Wedge(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        Solid S;

        if (n == 15 || n == 18)
        {
            var LocalP = new Point3D(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]));
            var LocalN = new Direction3D(theDI.Atof(a[5]), theDI.Atof(a[6]), theDI.Atof(a[7]));
            var LocalVx = new Direction3D(theDI.Atof(a[8]), theDI.Atof(a[9]), theDI.Atof(a[10]));
            var Axis = new Frame3D(LocalP, LocalN, LocalVx);
            if (n == 15)
            {
                S = new MakeWedge(Axis, theDI.Atof(a[11]), theDI.Atof(a[12]), theDI.Atof(a[13]), theDI.Atof(a[14])).Solid();
            }
            else
            {
                S = new MakeWedge(Axis,
                                              theDI.Atof(a[11]),
                                              theDI.Atof(a[12]),
                                              theDI.Atof(a[13]),
                                              theDI.Atof(a[14]),
                                              theDI.Atof(a[15]),
                                              theDI.Atof(a[16]),
                                              theDI.Atof(a[17])).Solid();
            }
        }
        else if (n == 6)
        {
            S = new MakeWedge(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5])).Solid();
        }
        else if (n == 9)
        {
            S = new MakeWedge(theDI.Atof(a[2]),
                                          theDI.Atof(a[3]),
                                          theDI.Atof(a[4]),
                                          theDI.Atof(a[5]),
                                          theDI.Atof(a[6]),
                                          theDI.Atof(a[7]),
                                          theDI.Atof(a[8])).Solid();
        }
        else
        {
            return 1;
        }

        theDI.SetShape(a[1], S);
        return 0;
    }

    private static int Cylinder(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Solid S;
        PlaneSurface? P = GetPlane(theDI, a[2]);

        if (n == 4)
        {
            S = new ApiMakeCylinder(theDI.Atof(a[2]), theDI.Atof(a[3])).Solid();
        }
        else if (n == 5)
        {
            if (P is null)
            {
                S = new ApiMakeCylinder(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]) * DegToRad).Solid();
            }
            else
            {
                S = new ApiMakeCylinder(P.Pln().Position().Ax2(), theDI.Atof(a[3]), theDI.Atof(a[4])).Solid();
            }
        }
        else if (n == 6)
        {
            if (P is null)
            {
                return 1;
            }

            S = new ApiMakeCylinder(P.Pln().Position().Ax2(), theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]) * DegToRad).Solid();
        }
        else
        {
            return 1;
        }

        theDI.SetShape(a[1], S);
        return 0;
    }

    private static int Cone(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Solid S;
        PlaneSurface? P = GetPlane(theDI, a[2]);

        if (n == 5)
        {
            S = new ApiMakeCone(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4])).Solid();
        }
        else if (n == 6)
        {
            if (P is null)
            {
                S = new ApiMakeCone(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]) * DegToRad).Solid();
            }
            else
            {
                S = new ApiMakeCone(P.Pln().Position().Ax2(), theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5])).Solid();
            }
        }
        else if (n == 7)
        {
            S = new ApiMakeCone(NonNull(P).Pln().Position().Ax2(),
                                         theDI.Atof(a[3]),
                                         theDI.Atof(a[4]),
                                         theDI.Atof(a[5]),
                                         theDI.Atof(a[6]) * DegToRad).Solid();
        }
        else
        {
            return 1;
        }

        theDI.SetShape(a[1], S);
        return 0;
    }

    private static int Sphere(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Solid S;
        PlaneSurface? P = GetPlane(theDI, a[2]);

        if (n == 3)
        {
            S = new MakeSphere(theDI.Atof(a[2])).Solid();
        }
        else if (n == 4)
        {
            if (P is null)
            {
                S = new MakeSphere(theDI.Atof(a[2]), theDI.Atof(a[3]) * DegToRad).Solid();
            }
            else
            {
                S = new MakeSphere(P.Pln().Position().Ax2(), theDI.Atof(a[3])).Solid();
            }
        }
        else if (n == 5)
        {
            if (P is null)
            {
                S = new MakeSphere(theDI.Atof(a[2]), theDI.Atof(a[3]) * DegToRad, theDI.Atof(a[4]) * DegToRad).Solid();
            }
            else
            {
                S = new MakeSphere(P.Pln().Position().Ax2(), theDI.Atof(a[3]), theDI.Atof(a[4]) * DegToRad).Solid();
            }
        }
        else if (n == 6)
        {
            if (P is null)
            {
                S = new MakeSphere(theDI.Atof(a[2]),
                                               theDI.Atof(a[3]) * DegToRad,
                                               theDI.Atof(a[4]) * DegToRad,
                                               theDI.Atof(a[5]) * DegToRad).Solid();
            }
            else
            {
                S = new MakeSphere(P.Pln().Position().Ax2(),
                                               theDI.Atof(a[3]),
                                               theDI.Atof(a[4]) * DegToRad,
                                               theDI.Atof(a[5]) * DegToRad).Solid();
            }
        }
        else if (n == 7)
        {
            S = new MakeSphere(NonNull(P).Pln().Position().Ax2(),
                                           theDI.Atof(a[3]),
                                           theDI.Atof(a[4]) * DegToRad,
                                           theDI.Atof(a[5]) * DegToRad,
                                           theDI.Atof(a[6]) * DegToRad).Solid();
        }
        else
        {
            return 1;
        }

        theDI.SetShape(a[1], S);
        return 0;
    }

    private static int Torus(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Solid S;
        PlaneSurface? P = GetPlane(theDI, a[2]);

        if (n == 4)
        {
            S = new MakeTorus(theDI.Atof(a[2]), theDI.Atof(a[3])).Solid();
        }
        else if (n == 5)
        {
            if (P is null)
            {
                S = new MakeTorus(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]) * DegToRad).Solid();
            }
            else
            {
                S = new MakeTorus(P.Pln().Position().Ax2(), theDI.Atof(a[3]), theDI.Atof(a[4])).Solid();
            }
        }
        else if (n == 6)
        {
            if (P is null)
            {
                S = new MakeTorus(theDI.Atof(a[2]),
                                              theDI.Atof(a[3]),
                                              theDI.Atof(a[4]) * DegToRad,
                                              theDI.Atof(a[5]) * DegToRad).Solid();
            }
            else
            {
                S = new MakeTorus(P.Pln().Position().Ax2(),
                                              theDI.Atof(a[3]),
                                              theDI.Atof(a[4]),
                                              theDI.Atof(a[5]) * DegToRad).Solid();
            }
        }
        else if (n == 7)
        {
            if (P is null)
            {
                S = new MakeTorus(theDI.Atof(a[2]),
                                              theDI.Atof(a[3]),
                                              theDI.Atof(a[4]) * DegToRad,
                                              theDI.Atof(a[5]) * DegToRad,
                                              theDI.Atof(a[6]) * DegToRad).Solid();
            }
            else
            {
                S = new MakeTorus(P.Pln().Position().Ax2(),
                                              theDI.Atof(a[3]),
                                              theDI.Atof(a[4]),
                                              theDI.Atof(a[5]) * DegToRad,
                                              theDI.Atof(a[6]) * DegToRad).Solid();
            }
        }
        else if (n == 8)
        {
            S = new MakeTorus(NonNull(P).Pln().Position().Ax2(),
                                          theDI.Atof(a[3]),
                                          theDI.Atof(a[4]),
                                          theDI.Atof(a[5]) * DegToRad,
                                          theDI.Atof(a[6]) * DegToRad,
                                          theDI.Atof(a[7]) * DegToRad).Solid();
        }
        else
        {
            return 1;
        }

        theDI.SetShape(a[1], S);
        return 0;
    }

    // BRepTest_TopologyCommands.cxx
    private static int HalfSpace(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 6)
        {
            return 1;
        }

        // The point on the side of the matter.
        var RefPnt = new Point3D(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));

        Shape Face = theDI.GetShape(a[2], global::Kerf.Topology.ShapeType.Face);
        if (Face.IsNull)
        {
            Shape Shell = theDI.GetShape(a[2], global::Kerf.Topology.ShapeType.Shell);
            if (Shell.IsNull)
            {
                di.Append(a[2]).Append(" must be a face or a shell\n");
                return 1;
            }

            var Half = new MakeHalfSpace(ShapeCasts.Shell(Shell), RefPnt);
            if (Half.IsDone)
            {
                theDI.SetShape(a[1], Half.Solid());
            }
            else
            {
                di.Append(" HalfSpace NotDone\n");
                return 1;
            }
        }
        else
        {
            var Half = new MakeHalfSpace(ShapeCasts.Face(Face), RefPnt);
            if (Half.IsDone)
            {
                theDI.SetShape(a[1], Half.Solid());
            }
            else
            {
                di.Append(" HalfSpace NotDone\n");
                return 1;
            }
        }

        return 0;
    }
}
