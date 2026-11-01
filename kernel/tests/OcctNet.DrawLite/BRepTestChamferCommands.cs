// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Occt.ChFiDS_ChamfMode;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.DrawLite;

/// <summary>
/// BRepTest's chamfer commands (BRepTest_ChamferCommands.cxx, unit 7N3): <c>chamf</c>, <c>chamf_throat</c> and
/// <c>chamf_throat_with_penetration</c>, written as OCCT writes them.
/// </summary>
internal static class BRepTestChamferCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "chamf", chamfer);
        DrawCxx.Add(theInterpreter, "chamf_throat", chamf_throat);
        DrawCxx.Add(theInterpreter, "chamf_throat_with_penetration", chamf_throat_with_penetration);
    }

    //===============================================================================
    // function : chamf_throat_with_penetration
    // purpose  : command to construct chamfers with constant throat with penetration
    //            on several edges
    //===============================================================================

    private static int chamf_throat_with_penetration(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 7)
        {
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[2]);
        if (S.IsNull())
        {
            return 1;
        }

        int i = 3;
        const int NbArg = 4;

        BRepFilletAPI_MakeChamfer aMCh = new(S);
        aMCh.SetMode(ChFiDS_ConstThroatWithPenetrationChamfer);

        while (i + NbArg <= narg)
        {
            TopoDS_Shape aLocalEdge = theDI.GetShape(a[i], TopAbs_EDGE);
            TopoDS_Edge E = TopoDS.Edge(aLocalEdge);
            TopoDS_Shape aLocalFace = theDI.GetShape(a[i + 1], TopAbs_FACE);
            TopoDS_Face F = TopoDS.Face(aLocalFace);
            if (!E.IsNull() && !F.IsNull() && (aMCh.Contour(E) == 0))
            {
                double offset = theDI.Atof(a[i + 2]);
                double throat = theDI.Atof(a[i + 3]);

                if (offset > Precision.Confusion() && throat > offset)
                {
                    aMCh.Add(offset, throat, E, F);
                }
            }
            i += NbArg;
        }

        return BuildAndSet(theDI, di, a, aMCh);
    }

    // compute the chamfer and display the result
    private static int BuildAndSet(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a, BRepFilletAPI_MakeChamfer aMCh)
    {
        if (aMCh.NbContours() == 0)
        {
            di.Append("No suitable edges to chamfer\n");
            return 1;
        }
        aMCh.Build();

        if (aMCh.IsDone())
        {
            theDI.SetShape(a[1], aMCh.Shape());
            return 0;
        }
        di.Append("compute of chamfer failed\n");
        return 1;
    }

    //===============================================================================
    // function : chamf_throat
    // purpose  : command to construct chamfers with constant throat on several edges
    //===============================================================================

    private static int chamf_throat(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 5)
        {
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[2]);
        if (S.IsNull())
        {
            return 1;
        }

        int i = 3;

        BRepFilletAPI_MakeChamfer aMCh = new(S);
        aMCh.SetMode(ChFiDS_ConstThroatChamfer);

        while (i + 1 < narg)
        {
            TopoDS_Shape aLocalEdge = theDI.GetShape(a[i], TopAbs_EDGE);
            TopoDS_Edge E = TopoDS.Edge(aLocalEdge);
            if (!E.IsNull() && (aMCh.Contour(E) == 0))
            {
                double throat = theDI.Atof(a[i + 1]);

                if (throat > Precision.Confusion())
                {
                    aMCh.Add(throat, E);
                }
            }
            i += 2;
        }

        return BuildAndSet(theDI, di, a, aMCh);
    }

    //=========================================================================
    // function : chamfer
    // purpose  : command to construct chamfers on several edges
    //=========================================================================

    private static int chamfer(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        // check the argument number of the command
        if (narg == 1)
        {
            di.Append(" help for chamf : \n");
            di.Append("   Construction by equal distances from edge          :  chamf newname shape edge dist\n");
            di.Append("   Construction by two distances from edge            :  chamf newname shape edge face dist1 dist2\n");
            di.Append("   Construction by distance from edge and given angle :  chamf newname shape edge face A dist angle\n");
            return 0;
        }
        if (narg < 5)
        {
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[2]);
        if (S.IsNull())
        {
            return 1;
        }

        int i = 3;

        BRepFilletAPI_MakeChamfer aMCh = new(S);

        while (i + 1 < narg)
        {
            TopoDS_Shape aLocalEdge = theDI.GetShape(a[i], TopAbs_EDGE);
            if (aLocalEdge.IsNull())
            {
                return 1;
            }
            TopoDS_Edge E = TopoDS.Edge(aLocalEdge);
            TopoDS_Shape aLocalFace = theDI.GetShape(a[i + 1], TopAbs_FACE);
            if (aLocalFace.IsNull())
            {
                // symmetric chamfer (one distance)
                double d1 = Atof(a[i + 1]); // C's atof, not Draw::Atof
                if (aMCh.Contour(E) == 0 && d1 > Precision.Confusion())
                {
                    aMCh.Add(d1, E);
                }
                i += 2;
            }
            else
            {
                TopoDS_Face F = TopoDS.Face(aLocalFace);

                if (i + 3 < narg)
                {
                    if (string.Equals(a[i + 2], "A", StringComparison.OrdinalIgnoreCase) && i + 4 < narg)
                    {
                        // chamfer with distance and angle
                        double d1 = theDI.Atof(a[i + 3]);
                        double angle = theDI.Atof(a[i + 4]);
                        angle *= Math.PI / 180.0;
                        if (aMCh.Contour(E) == 0 && d1 > Precision.Confusion() && angle > Precision.Confusion()
                            && Math.PI / 2 - angle > Precision.Confusion())
                        {
                            aMCh.AddDA(d1, angle, E, F);
                        }
                        i += 5;
                    }
                    else
                    {
                        // chamfer with two distances
                        double d1 = theDI.Atof(a[i + 2]);
                        double d2 = theDI.Atof(a[i + 3]);
                        if (aMCh.Contour(E) == 0 && d1 > Precision.Confusion() && d2 > Precision.Confusion())
                        {
                            aMCh.Add(d1, d2, E, F);
                        }
                        i += 4;
                    }
                }
                else
                {
                    // OCCT loops for ever here (i is not advanced: an edge and a face with too few words after
                    // them); Draw-lite stops reading the words (decision 1146)
                    break;
                }
            }
        }

        return BuildAndSet(theDI, di, a, aMCh);
    }

    // C's atof: the longest prefix that is a number, 0 if none.
    private static double Atof(string theText)
    {
        string t = theText.TrimStart();
        for (int n = t.Length; n > 0; n--)
        {
            if (double.TryParse(t.AsSpan(0, n), System.Globalization.NumberStyles.Float,
                                System.Globalization.CultureInfo.InvariantCulture, out double v))
            {
                return v;
            }
        }
        return 0.0;
    }
}
